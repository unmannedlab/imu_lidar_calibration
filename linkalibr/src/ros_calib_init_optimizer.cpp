//
// Created by usl on 3/6/21.
//

#include  <iostream>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>


#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <imuPacket/imuPacket.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_cost_function.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ostream>
#include <fstream>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

typedef message_filters::sync_policies::ApproximateTime
        <imuPacket::imuPacket,
         geometry_msgs::PoseStamped> SyncPolicy;

/// GTSAM Factor
using gtsam::symbol_shorthand::R; // Rotation

class HECFactor : public gtsam::NoiseModelFactor1<gtsam::Rot3> {
private:
    gtsam::Point3 m_axis_I_;
    gtsam::Point3 m_axis_L_;

public:
    HECFactor(gtsam::Key i, gtsam::Point3 axis_I, gtsam::Point3 axis_L, const gtsam::SharedNoiseModel& model) :
    gtsam::NoiseModelFactor1<gtsam::Rot3>(model, i), m_axis_I_(axis_I), m_axis_L_(axis_L) {}

    gtsam::Vector evaluateError(const gtsam::Rot3& I_R_L, boost::optional<gtsam::Matrix&> H = boost::none) const {
        gtsam::Matrix H_Rp_R, H_Rp_p;
        gtsam::Point3 error = m_axis_I_ - I_R_L.rotate(m_axis_L_, H_Rp_R, H_Rp_p);
        if(H)
            (*H) = (gtsam::Matrix(3, 3) << -H_Rp_R).finished();
        return (gtsam::Vector(3) << error.x(), error.y(), error.z()).finished();
    }
};

class calibInitOptimizer {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<imuPacket::imuPacket> *imupacket_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt;
    double accelerometer_noise_density;
    double gyroscope_noise_density;

    int no_of_frames = 0;
    int max_frames = 500;

    std::string calibration_result_filename;

    /// GTSAM stuff
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_values;
    gtsam::noiseModel::Diagonal::shared_ptr rotationNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3(1, 1, 1)));

public:
    calibInitOptimizer(ros::NodeHandle n) {
        nh = n;
        imupacket_sub = new message_filters::Subscriber<imuPacket::imuPacket>(nh, "/imu_packet", 1);
        pose_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/lidar_odometry", 1);

        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *imupacket_sub, *pose_sub);
        sync->registerCallback(boost::bind(&calibInitOptimizer::callback, this, _1, _2));

        accelerometer_noise_density = readParam<double>(nh, "accelerometer_noise_density");
        gyroscope_noise_density = readParam<double>(nh, "gyroscope_noise_density");
        max_frames = readParam<int>(nh, "max_frames");
        calibration_result_filename = readParam<std::string>(nh, "calibration_result_filename");

        double imuGravity = 9.81;
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(accelerometer_noise_density, 2);
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(gyroscope_noise_density, 2); //
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
        imuIntegratorOpt = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM( "Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void callback(const imuPacket::imuPacket::ConstPtr& imupacket_msg,
                  const geometry_msgs::PoseStamped ::ConstPtr& pose_msg) {
        int stamp_size = imupacket_msg->stamps.size();
        int accelreadings_size = imupacket_msg->accelreadings.size();
        int gyroreadings_size = imupacket_msg->gyroreadings.size();
        assert(stamp_size == accelreadings_size);
        assert(accelreadings_size == gyroreadings_size);
        for(int i = 1; i < stamp_size; i++) {
            double dt = imupacket_msg->stamps[i].toSec() - imupacket_msg->stamps[i-1].toSec();
            gtsam::Vector3 omega1 = gtsam::Vector3(imupacket_msg->gyroreadings[i-1].x,
                                                   imupacket_msg->gyroreadings[i-1].y,
                                                   imupacket_msg->gyroreadings[i-1].z);
            gtsam::Vector3 omega2 = gtsam::Vector3(imupacket_msg->gyroreadings[i].x,
                                                  imupacket_msg->gyroreadings[i].y,
                                                  imupacket_msg->gyroreadings[i].z);

            gtsam::Vector3 accel1 = gtsam::Vector3(imupacket_msg->accelreadings[i-1].x,
                                                   imupacket_msg->accelreadings[i-1].y,
                                                   imupacket_msg->accelreadings[i-1].z);
            gtsam::Vector3 accel2 = gtsam::Vector3(imupacket_msg->accelreadings[i].x,
                                                  imupacket_msg->accelreadings[i].y,
                                                  imupacket_msg->accelreadings[i].z);

            imuIntegratorOpt->integrateMeasurement(0.5*(accel1+accel2), 0.5*(omega1+omega2), dt);
        }

        Eigen::Matrix3d deltaR_I = imuIntegratorOpt->deltaRij().matrix();
        Eigen::Quaterniond quat_L;
        quat_L.x() = pose_msg->pose.orientation.x;
        quat_L.y() = pose_msg->pose.orientation.y;
        quat_L.z() = pose_msg->pose.orientation.z;
        quat_L.w() = pose_msg->pose.orientation.w;
        Eigen::Matrix3d deltaR_L(quat_L);

        Eigen::Vector3d axisAngle_lidar;
        Eigen::Vector3d axisAngle_imu;
        ceres::RotationMatrixToAngleAxis(deltaR_L.data(), axisAngle_lidar.data());
        ceres::RotationMatrixToAngleAxis(deltaR_I.data(), axisAngle_imu.data());

        /// GTSAM stuff
        graph.add(boost::make_shared<HECFactor>(R(0), gtsam::Point3(axisAngle_imu.x(),axisAngle_imu.y(),axisAngle_imu.z()),
                gtsam::Point3(axisAngle_lidar.x(), axisAngle_lidar.y(), axisAngle_lidar.z()), rotationNoise));
        ROS_INFO_STREAM("Frame: " << no_of_frames << " / " << max_frames);
        if(no_of_frames == max_frames) {
            solve();
        }
        no_of_frames++;
        imuIntegratorOpt->resetIntegration();
    }

    void solve() {
        gtsam::Rot3 priorRot = gtsam::Rot3::identity();
        initial_values.insert(R(0), priorRot);
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial_values).optimize();
        gtsam::Rot3 finalResult = result.at<gtsam::Rot3>(R(0));
        gtsam::Marginals marginals(graph, result);
        std::cout << "Rot3: \n" << std::endl;
        std::cout << finalResult.matrix() << std::endl;
        std::cout << "Euler Angles: " << finalResult.matrix().eulerAngles(0, 1, 2).transpose()*180/M_PI << std::endl;
        std::cout << "Marginal Covariance" << std::endl;
        std::cout << marginals.marginalCovariance(R(0)) << std::endl;

        std::ofstream result_file;
        result_file.open(calibration_result_filename.c_str());
        Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
        I_T_L.block(0, 0, 3, 3) = finalResult.matrix();
        I_T_L.block(0, 3, 3, 1) = Eigen::Vector3d::Zero();
        result_file << I_T_L;
        result_file.close();

        ros::shutdown();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_calib_init_optimizer");
    ros::NodeHandle nh("~");
    calibInitOptimizer cIO(nh);
    ros::spin();
}
