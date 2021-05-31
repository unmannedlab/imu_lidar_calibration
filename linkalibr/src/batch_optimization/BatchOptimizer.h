//
// Created by usl on 3/15/21.
//

#ifndef LINKALIBR_BATCHOPTIMIZER_H
#define LINKALIBR_BATCHOPTIMIZER_H

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/ISAM2.h>

#include "LoadPriorKnowledge.h"
//#include "PointToPlaneFactor.h"
//#include "PointToPlaneFactor2.h"
#include "PointToPlaneFactor3.h"

namespace lin_estimator {
    using gtsam::symbol_shorthand::N; // Navstate
    using gtsam::symbol_shorthand::X; // Pose
    using gtsam::symbol_shorthand::V; // Velocity
    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz) // IMU (+ Gyro) Biases
    using gtsam::symbol_shorthand::C; // Extrinsic Calibration // Pose3

    class BatchOptimizer {
    public:
        BatchOptimizer(OptimizerParams params) {
            std::cout << "-------- Batch Optimizer Started --------" << std::endl;
            prior_knowledge_loader = LoadPriorKnowledge(params);
            prior_knowledge_loader.GetIMUMeasurements(imu_measurements_);
            prior_knowledge_loader.GetIMUTrajectory(imu_trajectory_);
            prior_knowledge_loader.GetPlaneParams(plane_params_);
            prior_knowledge_loader.GetPlanarPoints(planar_points_);
            prior_knowledge_loader.GetInitialCalib(I_T_L_);
            std::cout << "-------- Batch Optimizer Loaded all Prior Knowledge --------" << std::endl;

            accelerometer_noise_density_ = params.accelerometer_noise_density;
            accelerometer_random_walk_ = params.accelerometer_random_walk;
            gyroscope_noise_density_ = params.gyroscope_noise_density;
            gyroscope_random_walk_ = params.gyroscope_random_walk;

            boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity);
            p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(accelerometer_noise_density_, 2);
            p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(gyroscope_noise_density_, 2); //
            p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
            gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

            prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
            prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
            prior_bias_noise_  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
//            prior_calib_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.10, 0.10, 0.10).finished());
            prior_calib_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
            correction_noise_imu_poseF2F_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.025, 0.025, 0.025, 0.05, 0.05, 0.05).finished());
//            correction_noise_imu_poseF2M_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.10, 0.10, 0.10).finished());
            correction_noise_imu_poseF2M_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());

            prior_navstate_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(9) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e4, 1e4, 1e4).finished());
            correction_noise_imu_pose_vel_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(9) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 1e4, 1e4, 1e4).finished());

            correction_noise_planar_constraint_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 1).finished());
            noise_model_between_bias_ = (gtsam::Vector(6) << accelerometer_random_walk_, accelerometer_random_walk_, accelerometer_random_walk_, gyroscope_random_walk_, gyroscope_random_walk_, gyroscope_random_walk_).finished();

            imu_integrator_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
            imu_integrator_planar_constraint_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

            graph_factors_ = gtsam::NonlinearFactorGraph();
            graph_values_ = gtsam::Values();

            Eigen::Matrix4d I_T_L;
            prior_knowledge_loader.GetInitialCalib(I_T_L);
            Eigen::Matrix3d I_R_L = I_T_L.block(0, 0, 3, 3);
            Eigen::Quaterniond I_quat_L(I_R_L);
            Eigen::Vector3d I_t_L = I_T_L.block(0, 3, 3, 1);
            imu_T_lidar = gtsam::Pose3(gtsam::Rot3(I_quat_L.w(), I_quat_L.x(), I_quat_L.y(), I_quat_L.z()),
                                     gtsam::Point3(I_t_L.x(), I_t_L.y(), I_t_L.z()));
            prev_calib_pose_ = imu_T_lidar;
            prev_pose_ = gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Vector3::Zero());
            prev_vel_ = gtsam::Vector3::Zero();
            prev_bias_ = gtsam::imuBias::ConstantBias();

            lidar_T_imu = imu_T_lidar.inverse();
            deskewed_points_csv_writer.open(params.planar_points_deskewed_csv_filename);
            imu_trajectory_out_csv_writer.open(params.imu_trajectory_out_csv_filename);
            init_calib_csv_writer.open(params.init_calib_csv_filename);
            final_calib_csv_writer.open(params.final_calib_csv_filename);
            imu_velocity_out_csv_writer.open(params.imu_velocity_out_csv_filename);

            opt_parameters.relinearizeThreshold = 0.1;
            opt_parameters.relinearizeSkip = 1;

            isam2_optimizer = gtsam::ISAM2(opt_parameters);
//            AddTrajConstraints();
            AddPlaneConstraints();
//            Solve();
            LogTrajectory();
            ValidationUsingMap();
        }

    private:
        void GetIMUMeasurementBetweenNodes(const uint64_t time0, const uint64_t time1,
                                           Eigen::aligned_vector<ImuMeasurement>& imu_measurements);
        void PreIntegrateIMUMeasurements(const Eigen::aligned_vector<ImuMeasurement>& imu_measurements,
                                         gtsam::PreintegratedImuMeasurements& pre_integrated_imu_measurements);

        bool FailureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

        void AddTrajConstraints();

        void AddPlaneConstraints();

        void GetMeasurementCovariance(const gtsam::Pose3 I1_T_Ik, const gtsam::Vector3 I1_v_Ik,
                                      const imuBias::ConstantBias& Bm, const gtsam::Pose3 Tc,
                                      const gtsam::Vector4 plane_param_measurement, const gtsam::Point3 lidar_point_measurement,
                                      const gtsam::Matrix66 P_I1TIk,
                                      const gtsam::Matrix33 P_I1vIk,
                                      const gtsam::Matrix66 P_bIk,
                                      const gtsam::Matrix66 P_Tc,
                                      double& measurement_covariance);

        void GetDeskewedMap(const uint64_t scan_ts, const uint64_t point_ts, const int scan_id, const int plane_id,
                            const gtsam::Pose3 I1_T_Ik, const gtsam::Vector3 I1_v_Ik, const imuBias::ConstantBias& Bm,
                            const gtsam::Pose3 Tc,
                            const lin_estimator::PreIntegratedIMUMeasurements preIntegratedImuMeasurements,
                            const gtsam::Point3 lidar_point_measurement);

        void LogTrajectory();

        void ValidationUsingMap();

        void Solve();

    private:
        LoadPriorKnowledge prior_knowledge_loader;

        Eigen::aligned_vector<ImuMeasurement> imu_measurements_;
        std::map<size_t , PoseVel> imu_trajectory_;
        Eigen::aligned_vector<Eigen::Vector4d> plane_params_;
        Eigen::aligned_vector<PlanarPoint> planar_points_;
        Eigen::Matrix4d I_T_L_;

        double accelerometer_noise_density_;
        double accelerometer_random_walk_;
        double gyroscope_noise_density_;
        double gyroscope_random_walk_;

        const double imu_gravity = -9.81;

        gtsam::PreintegratedImuMeasurements *imu_integrator_;
        gtsam::PreintegratedImuMeasurements *imu_integrator_planar_constraint_;

        gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_navstate_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_calib_noise_;

        gtsam::noiseModel::Diagonal::shared_ptr correction_noise_imu_poseF2F_;
        gtsam::noiseModel::Diagonal::shared_ptr correction_noise_imu_poseF2M_;
        gtsam::noiseModel::Diagonal::shared_ptr correction_noise_imu_pose_vel_;
        gtsam::noiseModel::Diagonal::shared_ptr correction_noise_planar_constraint_;

        gtsam::Vector noise_model_between_bias_;

        gtsam::Pose3 prev_calib_pose_;
        gtsam::NavState prev_state_;
        gtsam::NavState curr_state_;
        gtsam::Pose3 prev_pose_;
        gtsam::Vector3 prev_vel_;
        gtsam::imuBias::ConstantBias prev_bias_;

        gtsam::Pose3 imu_T_lidar; // I_T_L
        gtsam::Pose3 lidar_T_imu; // L_T_I

        gtsam::Values optimization_result;
        int key = 0;

        std::ofstream deskewed_points_csv_writer;
        std::ofstream imu_trajectory_out_csv_writer;
        std::ofstream init_calib_csv_writer;
        std::ofstream final_calib_csv_writer;
        std::ofstream imu_velocity_out_csv_writer;

        gtsam::NonlinearFactorGraph graph_factors_;
        gtsam::Values graph_values_;
        gtsam::ISAM2 isam2_optimizer;
        gtsam::ISAM2Params opt_parameters;

        std::map<size_t, gtsam::Pose3> pose_vector_;
        std::map<size_t, gtsam::Vector3> velocity_vector_;
        std::map<size_t, gtsam::imuBias::ConstantBias> bias_vector_;
    };
}

#endif //LINKALIBR_BATCHOPTIMIZER_H
