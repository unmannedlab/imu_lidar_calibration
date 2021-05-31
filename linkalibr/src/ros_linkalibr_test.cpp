//
// Created by usl on 12/14/20.
//

#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include  <iostream>
#include "track/lidarOdometry.h"

#include "core/lincalibManagerOptions.h"
#include "core/lincalibManager.h"

#include "utils/parse_ros.h"
#include "utils/quat_ops.h"

#include <tf/transform_broadcaster.h>

#include "utils/pcl_utils.h"

#include "surfel_association/SurfelAssociation.h"

#include <pcl/visualization/cloud_viewer.h>

using namespace lin_estimator;

lincalibManager *sys;

void downSampleCloud(const pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud_in_pcl,
                     pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud_out_pcl,
                     int ring_downsample_factor) {
    cloud_out_pcl->header = cloud_in_pcl->header;
    cloud_out_pcl->is_dense = cloud_in_pcl->is_dense;
    cloud_out_pcl->height = cloud_in_pcl->height;
    cloud_out_pcl->width = cloud_in_pcl->width;
    cloud_out_pcl->resize(cloud_in_pcl->width*cloud_in_pcl->height);
    for(int h = 0; h < cloud_in_pcl->height; h++) {
        for(int w = 0; w < cloud_in_pcl->width; w++) {
            lin_core::PointXYZIR8Y cloud_point = cloud_in_pcl->at(w, h);
            /// Ignore non even rings
            if(cloud_point.ring%ring_downsample_factor != 0)
                continue;
            /// Ignore points with NaNs
            if(pcl_isnan(cloud_point.x) || pcl_isnan(cloud_point.y) || pcl_isnan(cloud_point.z)) {
                continue;
            }
            cloud_out_pcl->at(w, h) = cloud_point;
        }
    }
}

int main(int argc, char** argv) {
    /// Launch ros node
    ros::init(argc, argv, "ros_test_node");
    ros::NodeHandle nh("~");
    /// Create our lincalib system
    lincalibManagerOptions params = parse_ros_nodehandler(nh);
    sys = new lincalibManager(params);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/raw_cloud_out", 1);
    ros::Publisher imu_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_pose_out", 1);
    ros::Publisher imu_odom_pub = nh.advertise<nav_msgs::Odometry>("/imu_odom_out", 1);
    ros::Publisher lidar_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose_out", 1);
    ros::Publisher planar_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/planar_points_out", 1);
    ros::Publisher undistorted_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/undistorted_cloud_out", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_measurement_out", 1);
    ros::Publisher planar_param_first_scan_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/planar_param_first_scan", 1);
    ros::Publisher planar_param_curr_scan_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/planar_param_curr_scan", 1);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);

    ///===================================================================================
    ///===================================================================================
    ///===================================================================================

    /// Our topics (IMU and LIDAR)
    std::string topic_imu;
    std::string topic_lidar;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");

    /// Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/usl/datasets/ouster_vectornav.bag");
    ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());

    /// Get our start location and how much of the bag we want to play
    /// Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO_STREAM("bag start: " << bag_start);
    ROS_INFO_STREAM("bag duration: " << bag_durr);

    /// Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    /// We should load the bag as a view
    /// Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    /// Start a few seconds in from the full view time
    /// If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish =
            (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO_STREAM("Time start = " << time_init.toSec());
    ROS_INFO_STREAM("Time end = " << time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    /// Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    /// Buffer variables for our system (so we always have imu to use)
    pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud(new pcl::PointCloud<lin_core::PointXYZIR8Y>);
    pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud_downsampled(new pcl::PointCloud<lin_core::PointXYZIR8Y>);

    /// Broadcaster
    tf::TransformBroadcaster br1, br2, br3;
    tf::Transform transform1, transform2, transform3;
    std::string imu_frame_name, lidar_frame_name, lidar_first_frame;
    ///===================================================================================
    ///===================================================================================
    ///===================================================================================
    // Step through the rosbag
    int lidar_scan_no = 0;
    ros::Time start_time = ros::Time::now();
    int no_of_imu = 0;
    bool first_lidar_frame = true;
    Eigen::Matrix4d G_T_L1 = Eigen::Matrix4d::Identity();
    Eigen::Vector3d G_t_L1 = Eigen::Vector3d::Zero();
    Eigen::Quaterniond G_quat_L1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::string imu_trajectory_filename;
    std::ofstream imu_trajectory_csv;

    std::string lidar_trajectory_filename;
    std::ofstream lidar_trajectory_csv;

    std::string imu_measurements_filename;
    std::ofstream imu_measurements_csv;

    if(params.gen_data_for_optimization) {
        nh.param<std::string>("imu_measurements_filename", imu_measurements_filename, "/home/usl/catkin_ws/src/linkalibr/data/imu_measurements.csv");
        imu_measurements_csv.open(imu_measurements_filename);

        nh.param<std::string>("imu_trajectory_filename", imu_trajectory_filename, "/home/usl/catkin_ws/src/linkalibr/data/imu_trajectory.csv");
        imu_trajectory_csv.open(imu_trajectory_filename);

        nh.param<std::string>("lidar_trajectory_filename", lidar_trajectory_filename, "/home/usl/catkin_ws/src/linkalibr/data/lidar_trajectory.csv");
        lidar_trajectory_csv.open(lidar_trajectory_filename);
    }
    bool first_lodom = true;
    for (const rosbag::MessageInstance& m : view) {
        /// If ROS wants us to stop, break out
        if(!ros::ok())
            break;
        /// Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
        if (s_imu != nullptr && m.getTopic() == topic_imu) {
            imu_pub.publish(s_imu);
            imu_frame_name = s_imu->header.frame_id;
            double time_imu = (*s_imu).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s_imu).angular_velocity.x, (*s_imu).angular_velocity.y, (*s_imu).angular_velocity.z;
            am << (*s_imu).linear_acceleration.x, (*s_imu).linear_acceleration.y, (*s_imu).linear_acceleration.z;
            if(params.gen_data_for_optimization) {
                imu_measurements_csv << s_imu->header.stamp.toNSec() << ", "
                << am.x() << ", " << am.y() << ", " << am.z()<< ", "
                << wm.x() << ", " << wm.y() << ", " << wm.z() << "\n";
            }
            State* state = sys->get_state();
            Eigen::Matrix<double,13,1> state_plus = Eigen::Matrix<double,13,1>::Zero();
            sys->get_propagator()->fast_state_propagate(state, time_imu, state_plus);

            // Our odometry message
            nav_msgs::Odometry odomIinM;
            odomIinM.header.stamp = (*s_imu).header.stamp;
            odomIinM.header.frame_id = "map";

            // The POSE component (orientation and position)
            odomIinM.pose.pose.orientation.x = state_plus(0);
            odomIinM.pose.pose.orientation.y = state_plus(1);
            odomIinM.pose.pose.orientation.z = state_plus(2);
            odomIinM.pose.pose.orientation.w = state_plus(3);
            odomIinM.pose.pose.position.x = state_plus(4);
            odomIinM.pose.pose.position.y = state_plus(5);
            odomIinM.pose.pose.position.z = state_plus(6);

            // Finally set the covariance in the message (in the order position then orientation as per ros convention)
            std::vector<Type*> statevars;
            statevars.push_back(state->_imu->pose()->p());
            statevars.push_back(state->_imu->pose()->q());
            Eigen::Matrix<double,6,6> covariance_posori = StateHelper::get_marginal_covariance(sys->get_state(),statevars);
            for(int r=0; r<6; r++) {
                for(int c=0; c<6; c++) {
                    odomIinM.pose.covariance[6*r+c] = covariance_posori(r,c);
                }
            }

            // The TWIST component (angular and linear velocities)
            odomIinM.child_frame_id = "imu";
            odomIinM.twist.twist.linear.x = state_plus(7);
            odomIinM.twist.twist.linear.y = state_plus(8);
            odomIinM.twist.twist.linear.z = state_plus(9);
            odomIinM.twist.twist.angular.x = state_plus(10); // we do not estimate this...
            odomIinM.twist.twist.angular.y = state_plus(11); // we do not estimate this...
            odomIinM.twist.twist.angular.z = state_plus(12); // we do not estimate this...

            // Velocity covariance (linear then angular)
            statevars.clear();
            statevars.push_back(state->_imu->v());
            Eigen::Matrix<double,6,6> covariance_linang = INFINITY*Eigen::Matrix<double,6,6>::Identity();
            covariance_linang.block(0,0,3,3) = StateHelper::get_marginal_covariance(sys->get_state(),statevars);
            for(int r=0; r<6; r++) {
                for(int c=0; c<6; c++) {
                    odomIinM.twist.covariance[6*r+c] = (std::isnan(covariance_linang(r,c))) ? 0 : covariance_linang(r,c);
                }
            }

            // Finally, publish the resulting odometry message
            imu_odom_pub.publish(odomIinM);

            /// Send it to our linkalibr system
            sys->feed_measurement_imu(time_imu, wm, am);
            no_of_imu++;
        }

        /// Handle Lidar measurement
        sensor_msgs::PointCloud2::ConstPtr s_lidar = m.instantiate<sensor_msgs::PointCloud2>();
        if (s_lidar != nullptr && m.getTopic() == topic_lidar) {
            lidar_frame_name = s_lidar->header.frame_id;
            ROS_INFO_STREAM("No of imu measurements: " << no_of_imu);
            no_of_imu = 0;
            cloud_pub.publish(*s_lidar);
            double time_lidar = (*s_lidar).header.stamp.toSec();
            pcl::fromROSMsg(*s_lidar, *cloud);
            /// TODO :  Try Downsampling here
//            downSampleCloud(cloud, cloud_downsampled, 1);
            /// Send it to linkalibr system
            sys->feed_measurement_lidar(time_lidar, cloud);
            VPointCloud cloud_undistorted = sys->get_undistorted_cloud();
            sensor_msgs::PointCloud2 cloud_undistorted_ros;
            pcl::toROSMsg(cloud_undistorted, cloud_undistorted_ros);
            cloud_undistorted_ros.header.frame_id = s_lidar->header.frame_id;
            cloud_undistorted_ros.header.stamp = s_lidar->header.stamp;
            cloud_undistorted_ros.header.seq = s_lidar->header.seq;
            undistorted_cloud_pub.publish(cloud_undistorted_ros);

            ROS_INFO_STREAM("Lidar scan no: " << lidar_scan_no);
            lidar_scan_no++;
            State* state_k = sys->get_state();
            Eigen::Vector3d G_t_Ik = state_k->_imu->pos();
            Eigen::Vector4d Ik_quat_imu_G = state_k->_imu->quat();
            Eigen::Matrix3d Ik_R_G = lin_core::quat_2_Rot(Ik_quat_imu_G);
            Eigen::Matrix3d G_R_Ik = Ik_R_G.transpose();
            Eigen::Matrix4d G_T_Ik = Eigen::Matrix4d::Identity();
            G_T_Ik.block(0, 0, 3, 3) = G_R_Ik;
            G_T_Ik.block(0, 3, 3, 1) = G_t_Ik;
            Eigen::Quaterniond quat_imu_k_eig(G_R_Ik);
            transform1.setOrigin(tf::Vector3(G_t_Ik.x(), G_t_Ik.y(), G_t_Ik.z()));
            transform1.setRotation(tf::Quaternion(quat_imu_k_eig.x(), quat_imu_k_eig.y(), quat_imu_k_eig.z(), quat_imu_k_eig.w()));
            br1.sendTransform(tf::StampedTransform(transform1, (*s_lidar).header.stamp, "map", imu_frame_name));

            double t_ItoL = state_k->_calib_dt_LIDARtoIMU->value()(0);
            double timestamp_inI = state_k->_timestamp + t_ItoL;

            // Create pose of IMU (note we use the bag time)
            geometry_msgs::PoseWithCovarianceStamped poseIinG;
            poseIinG.header.stamp = ros::Time(timestamp_inI);
            poseIinG.header.frame_id = "map";
            poseIinG.pose.pose.orientation.x = quat_imu_k_eig.x();
            poseIinG.pose.pose.orientation.y = quat_imu_k_eig.y();
            poseIinG.pose.pose.orientation.z = quat_imu_k_eig.z();
            poseIinG.pose.pose.orientation.w = quat_imu_k_eig.w();
            poseIinG.pose.pose.position.x = G_t_Ik.x();
            poseIinG.pose.pose.position.y = G_t_Ik.y();
            poseIinG.pose.pose.position.z = G_t_Ik.z();
            // Finally set the covariance in the message (in the order position then orientation as per ros convention)
            std::vector<Type*> statevars;
            statevars.push_back(state_k->_imu->pose()->p());
            statevars.push_back(state_k->_imu->pose()->q());
            Eigen::Matrix<double,6,6> covariance_posori = StateHelper::get_marginal_covariance(sys->get_state(),statevars);
            for(int r=0; r<6; r++) {
                for(int c=0; c<6; c++) {
                    poseIinG.pose.covariance[6*r+c] = covariance_posori(r,c);
                }
            }
            imu_pose_pub.publish(poseIinG);

            ROS_INFO_STREAM("No of lodom: " << sys->get_track_lodom()->get_odom_data().size());
            int no_of_lodoms = sys->get_track_lodom()->get_odom_data().size();

            if(no_of_lodoms > 0) {
                Eigen::Matrix3d I_R_L = lin_core::quat_2_Rot(sys->get_state()->_calib_LIDARtoIMU->quat());
                Eigen::Vector3d I_t_L = sys->get_state()->_calib_LIDARtoIMU->pos();
                Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
                I_T_L.block(0, 0, 3, 3) = I_R_L;
                I_T_L.block(0, 3, 3, 1) = I_t_L;
                Eigen::Matrix4d G_T_I1 = sys->G_T_I1;
                if(first_lodom) {
                    G_T_I1 = sys->G_T_I1;
                    first_lodom = false;
                }

                Eigen::Matrix4d I1_T_Ik = G_T_I1.inverse()*G_T_Ik;
                Eigen::Vector3d I1_t_Ik = I1_T_Ik.block(0, 3, 3, 1);
                Eigen::Matrix3d I1_R_Ik = I1_T_Ik.block(0, 0, 3, 3);
                Eigen::Quaterniond I1_quat_Ik(I1_R_Ik);

                /// This the lidar pose in the L1 frame (not in G frame, where G is the world frame for system)
                Eigen::Matrix4d L1_T_Lk = sys->get_track_lodom()->get_current_odometry().pose;
                Eigen::Vector3d L1_t_Lk = L1_T_Lk.block(0, 3, 3, 1);
                Eigen::Matrix3d L1_R_Lk = L1_T_Lk.block(0, 0, 3, 3);
                Eigen::Quaterniond L1_quat_Lk(L1_R_Lk);
                if(params.gen_data_for_optimization) {
                    lidar_trajectory_csv << ros::Time(state_k->_timestamp).toNSec() << ", "
                                       << L1_quat_Lk.x() << ", "<< L1_quat_Lk.y() << ", " << L1_quat_Lk.z() << ", "<< L1_quat_Lk.w() << ", "
                                       << L1_t_Lk.x() << ", "<< L1_t_Lk.y() << ", " << L1_t_Lk.z() << "\n";
                    imu_trajectory_csv << ros::Time(state_k->_timestamp).toNSec() << ", "
                                         << I1_quat_Ik.x() << ", "<< I1_quat_Ik.y() << ", " << I1_quat_Ik.z() << ", "<< I1_quat_Ik.w() << ", "
                                         << I1_t_Ik.x() << ", "<< I1_t_Ik.y() << ", " << I1_t_Ik.z() << "\n";
                }
                geometry_msgs::PoseStamped pose_lidar;
                pose_lidar.header.stamp = (*s_lidar).header.stamp;
                pose_lidar.header.frame_id = (*s_lidar).header.frame_id;
                pose_lidar.pose.position.x = L1_t_Lk.x();
                pose_lidar.pose.position.y = L1_t_Lk.y();
                pose_lidar.pose.position.z = L1_t_Lk.z();
                pose_lidar.pose.orientation.x = L1_quat_Lk.x();
                pose_lidar.pose.orientation.y = L1_quat_Lk.y();
                pose_lidar.pose.orientation.z = L1_quat_Lk.z();
                pose_lidar.pose.orientation.w = L1_quat_Lk.w();
                lidar_pose_pub.publish(pose_lidar);

                /// The following is the lidar pose in G frame
//                Eigen::Matrix4d G_T_Lk = G_T_I1*I_T_L*L1_T_Lk;
                Eigen::Matrix4d G_T_Lk = G_T_Ik*I_T_L;
                Eigen::Matrix3d G_R_Lk = G_T_Lk.block(0, 0, 3, 3);
                Eigen::Vector3d G_t_Lk = G_T_Lk.block(0, 3, 3, 1);
                Eigen::Quaterniond G_quat_Lk(G_R_Lk);

                if(first_lidar_frame) {
                    first_lidar_frame = false;
                    G_T_L1 = G_T_Lk;
                    G_t_L1 = G_T_L1.block(0, 3, 3, 1);
                    Eigen::Matrix3d G_R_L1 = G_T_L1.block(0, 0, 3, 3);
                    G_quat_L1 = Eigen::Quaterniond(G_R_L1);
                }

                transform2.setOrigin(tf::Vector3(G_t_Lk.x(), G_t_Lk.y(), G_t_Lk.z()));
                transform2.setRotation(tf::Quaternion(G_quat_Lk.x(), G_quat_Lk.y(), G_quat_Lk.z(), G_quat_Lk.w()));
                br2.sendTransform(tf::StampedTransform(transform2, (*s_lidar).header.stamp, "map", lidar_frame_name));

                transform3.setOrigin(tf::Vector3(G_t_L1.x(), G_t_L1.y(), G_t_L1.z()));
                transform3.setRotation(tf::Quaternion(G_quat_L1.x(), G_quat_L1.y(), G_quat_L1.z(), G_quat_L1.w()));
                br3.sendTransform(tf::StampedTransform(transform3, (*s_lidar).header.stamp, "map", "os_lidar_1"));
            }
            if(sys->get_track_lodom()->isKeyFrame()) {
                map_cloud = sys->get_track_lodom()->getTargetMap();
                sensor_msgs::PointCloud2 map_cloud_ros;
                pcl::toROSMsg(*map_cloud, map_cloud_ros);
                map_cloud_ros.header.frame_id = "os_lidar_1";
                map_cloud_ros.header.stamp = s_lidar->header.stamp;
                map_pub.publish(map_cloud_ros);
            }
        }
    }

    if(params.gen_map_data) {
        std::cout << "Generating map pointcloud as csv.." << std::endl;
        std::string map_csv_file_name;
        nh.param<std::string>("map_csv_file_name", map_csv_file_name, "/home/usl/catkin_ws/src/linkalibr/data/map_csv_file.csv");
        std::ofstream map_csv_file;
        map_csv_file.open(map_csv_file_name);
        for(int i = 0; i < map_cloud->points.size(); i++) {
            map_csv_file << map_cloud->points[i].x << ", " << map_cloud->points[i].y << ", "  << map_cloud->points[i].z << "\n";
        }
        map_csv_file.close();
        std::cout <<"Stored map pointcloud as csv at: " << map_csv_file_name << std::endl;
    }

    if(params.gen_data_for_optimization) {
        imu_measurements_csv.close();
        lidar_trajectory_csv.close();
        imu_trajectory_csv.close();
    }
    ros::Time end_time = ros::Time::now();
    ROS_INFO_STREAM("Reached end of bag");
    ROS_INFO_STREAM("Kalman Filtering took : " << end_time.toSec() - start_time.toSec() << " [s]");
    /// Write the final I_T_L to a text file
    State* state = sys->get_state();
    Pose *calibration = state->_calib_LIDARtoIMU;
    Eigen::Matrix3d I_R_L = calibration->Rot();
    Eigen::Vector3d I_t_L = calibration->pos();
    Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
    I_T_L.block(0, 0, 3, 3) = I_R_L;
    I_T_L.block(0, 3, 3, 1) = I_t_L;
    std::cout << "Writing KF calibration result to: " << params.calibration_result_filename << std::endl;
    std::ofstream result_calibration;
    result_calibration.open(params.calibration_result_filename.c_str());
    result_calibration << I_T_L;
    result_calibration.close();

    Eigen::Vector3d eulerXYZ = I_R_L.eulerAngles(0, 1, 2)*180/M_PI;
    ROS_INFO_STREAM("Translation [in m]: " << I_t_L.transpose());
    ROS_INFO_STREAM("Euler Angles [in deg]: " << eulerXYZ.transpose());

    if(params.gen_data_for_optimization) {
        /// Batch Optimization stuff
        ros::Time time_surfAss_start = ros::Time::now();
        /// Surfel Association
        ROS_INFO_STREAM("[Starting Data Association]");
        lin_core::SurfelAssociation::Ptr surfelAssociation =
                std::make_shared<lin_core::SurfelAssociation>(0.05, 0.9);
        surfelAssociation->setSurfelMap(sys->get_track_lodom()->getNDTPtr(), sys->get_map_time());

        for(int i = 0; i < sys->get_track_lodom()->get_odom_data().size(); i++) {
            ROS_INFO_STREAM("[Surfel Association] for frame: " << i);
            std::string raw_scan_filename = params.raw_scan_folder_name+"/"+std::to_string(i+1)+".pcd";
            std::string deskewed_scaninMap_filename = params.deskewed_scan_folder_name+"/"+std::to_string(i+1)+".pcd";
            TPointCloud scan_raw;
            pcl::io::loadPCDFile<TPoint> (raw_scan_filename, scan_raw);
            VPointCloud scan_inM;
            pcl::io::loadPCDFile<VPoint> (deskewed_scaninMap_filename, scan_inM);
            uint64_t scan_timestamp = ros::Time(sys->get_track_lodom()->get_odom_data().at(i).timestamp).toNSec();
            surfelAssociation->getAssociation(scan_inM.makeShared(), scan_raw.makeShared(), scan_timestamp, i,10);
        }
        surfelAssociation->averageTimeDownSample(1);
        std::cout << "[Surfel Association] Number of surfel points: " << surfelAssociation->get_surfel_points().size() << std::endl;
        ros::Time time_surfAss_end = ros::Time::now();
        ROS_INFO_STREAM("[Surfel Association] Time taken in seconds for surfel association: " << time_surfAss_end.toSec() - time_surfAss_start.toSec());

        bool view_surfel_map;
        nh.param<bool>("view_surfel_map", view_surfel_map, false);

        std::string plane_params_filename;
        nh.param<std::string>("plane_params_filename", plane_params_filename, "/home/usl/catkin_ws/src/linkalibr/data/plane_params.csv");

        std::string planar_points_raw_scans_filename;
        nh.param<std::string>("planar_points_raw_scans_filename", planar_points_raw_scans_filename, "/home/usl/catkin_ws/src/linkalibr/data/planar_points_raw_scans.csv");
        std::ofstream planar_points_raw_scans_filename_csv;
        planar_points_raw_scans_filename_csv.open(planar_points_raw_scans_filename);

        std::string planar_points_deskewed_map_filename;
        nh.param<std::string>("planar_points_deskewed_map_filename", planar_points_deskewed_map_filename, "/home/usl/catkin_ws/src/linkalibr/data/planar_points_deskewed_map.csv");
        std::ofstream planar_points_deskewed_map_filename_csv;
        planar_points_deskewed_map_filename_csv.open(planar_points_deskewed_map_filename);

        for(int i = 0; i < surfelAssociation->get_surfel_points().size(); i++) {
            SurfelAssociation::SurfelPoint surfelPoint = surfelAssociation->get_surfel_points().at(i);
            planar_points_raw_scans_filename_csv << surfelPoint.scan_timestamp << "," << surfelPoint.point_timestamp << "," << surfelPoint.scan_id << ","
                                   << surfelPoint.plane_id << "," << surfelPoint.point.x() << "," << surfelPoint.point.y() << "," << surfelPoint.point.z() << "\n";
            planar_points_deskewed_map_filename_csv << surfelPoint.scan_timestamp << "," << surfelPoint.point_timestamp << "," << surfelPoint.scan_id << ","
                                   << surfelPoint.plane_id << "," << surfelPoint.point_in_map.x() << "," << surfelPoint.point_in_map.y() << "," << surfelPoint.point_in_map.z() << "\n";
        }

        Eigen::aligned_vector<SurfelAssociation::SurfelPlane> surfel_planes_ = surfelAssociation->get_surfel_planes();
        lin_core::VPointCloud::Ptr surfel_points(new lin_core::VPointCloud);
        std::ofstream plane_params_csv_file;
        plane_params_csv_file.open(plane_params_filename);

        std::string surfel_map_filename;
        nh.param<std::string>("surfel_map_filename", surfel_map_filename, "/home/usl/catkin_ws/src/linkalibr/data/forOptimization/surfel_map.csv");
        std::ofstream surfel_map_csv;
        surfel_map_csv.open(surfel_map_filename);

        for(int i = 0; i < surfel_planes_.size(); i++) {
            plane_params_csv_file << surfel_planes_[i].p4.x() << "," << surfel_planes_[i].p4.y() << "," << surfel_planes_[i].p4.z() << "," << surfel_planes_[i].p4.w() << "\n";
            lin_core::VPointCloud inlier_points = surfel_planes_[i].cloud_inlier;
            for(int j = 0; j < inlier_points.size(); j++) {
                surfel_points->points.push_back(inlier_points.points[j]);
                surfel_map_csv << inlier_points.points[j].x << ", " << inlier_points.points[j].y << ", " << inlier_points.points[j].z << std::endl;
            }
        }
        surfel_map_csv.close();
        if(view_surfel_map) {
            pcl::visualization::CloudViewer viewer ("Surfel Viewer");
            viewer.showCloud(surfel_points, "surfel map");
            while (!viewer.wasStopped ()) {
            }
        }
    }

    return EXIT_SUCCESS;
}