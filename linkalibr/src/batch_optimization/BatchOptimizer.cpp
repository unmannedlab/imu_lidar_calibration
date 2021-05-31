//
// Created by usl on 3/15/21.
//

#include "BatchOptimizer.h"

namespace lin_estimator {

    void BatchOptimizer::GetDeskewedMap(const uint64_t scan_ts, const uint64_t point_ts, const int scan_id, const int plane_id,
                                        const gtsam::Pose3 I1_T_Ik, const gtsam::Vector3 I1_v_Ik, const imuBias::ConstantBias& Bm,
                                        const gtsam::Pose3 Tc,
                                        const lin_estimator::PreIntegratedIMUMeasurements preIntegratedImuMeasurements,
                                        const gtsam::Point3 lidar_point_measurement) {
        /// Pre-integrated measurements
        Rot3 deltaR = preIntegratedImuMeasurements.deltaR;
        Vector3 deltaP = preIntegratedImuMeasurements.deltaP;
        Vector3 deltaV = preIntegratedImuMeasurements.deltaV;
        double deltaT = preIntegratedImuMeasurements.deltaT;
        Vector3 gravity = preIntegratedImuMeasurements.gravity;
        Matrix93 H_bias_omega = preIntegratedImuMeasurements.H_bias_omega;
        Matrix93 H_bias_accel = preIntegratedImuMeasurements.H_bias_accel;

        Matrix33 H_delR_bias_accel = H_bias_accel.block(0, 0, 3, 3);
        Matrix33 H_delP_bias_accel = H_bias_accel.block(3, 0, 3, 3);
        Matrix33 H_delV_bias_accel = H_bias_accel.block(6, 0, 3, 3);
        Vector3 bias_accel_hat = Bm.accelerometer();
        Matrix33 H_delR_bias_omega = H_bias_omega.block(0, 0, 3, 3);
        Matrix33 H_delP_bias_omega = H_bias_omega.block(3, 0, 3, 3);
        Matrix33 H_delV_bias_omega = H_bias_omega.block(6, 0, 3, 3);
        Vector3 bias_omega_hat = Bm.gyroscope();


        Pose3 A = Pose3(Rot3::identity(), I1_v_Ik*deltaT + 0.5*gravity*deltaT*deltaT);
        Pose3 B11 = Pose3(Rot3::identity(), H_delP_bias_omega*bias_omega_hat);
        Pose3 B12 = Pose3(Rot3::identity(), H_delP_bias_accel*bias_accel_hat);
        Pose3 B2 = Pose3(deltaR, deltaP);
        Matrix3 H_exp_Hbg;
        Pose3 B3 = Pose3(gtsam::Rot3::Expmap(H_delR_bias_omega*bias_omega_hat, H_exp_Hbg), gtsam::Vector3::Zero());
        Matrix H_1, H_2, H_3, H_4, H_5, H_6, H_7, H_8, H_9, H_10, H_11, H_12, H_13, H_14, H_15;
        Pose3 L1_T_Lmplusi = Tc.inverse(H_1).compose(A, H_2, H_3).compose(I1_T_Ik, H_4, H_5).compose(B11, H_6, H_7).
                compose(B12, H_8, H_9).compose(B2, H_10, H_11).compose(B3, H_12, H_13).compose(Tc, H_14, H_15);

        /// Jacobian of L1_T_Lmplusi wrt I1_T_Ik
        Matrix66 H_L1TLmplusi_I1TIk = H_14*H_12*H_10*H_8*H_6*H_5;

        /// Jacobian of L1_T_Lmplusi wrt A
        Matrix66 H_L1Tmplusi_A = H_14*H_12*H_10*H_8*H_6*H_4*H_3;
        Matrix33 H_L1Rmplusi_pa = H_L1Tmplusi_A.block(0, 3, 3, 3);
        Matrix33 H_L1pmplusi_pa = H_L1Tmplusi_A.block(3, 3, 3, 3);

        /// Jacobian of L1_T_Lmplusi wrt Tc
        Matrix6 H_L1TLmplusi_Tc;
        H_L1TLmplusi_Tc = H_15 + H_14*H_12*H_10*H_8*H_6*H_4*H_2*H_1;

        /// Jacobian of L1_T_Lmplusi wrt I1_v_Ik
        Matrix63 H_L1TLmplusi_I1vIk; ;
        H_L1TLmplusi_I1vIk.block(0, 0, 3, 3) = H_L1Rmplusi_pa;
        H_L1TLmplusi_I1vIk.block(3, 0, 3, 3) = H_L1pmplusi_pa*deltaT;

        /// Jacobian of L1_T_Lmplusi wrt B12
        Matrix66 H_L1TLmplusi_B12 = H_14*H_12*H_10*H_9;
        Matrix33 H_L1RLmplusi_pB12 = H_L1TLmplusi_B12.block(0, 3, 3, 3);
        Matrix33 H_L1pLmplusi_pB12 = H_L1TLmplusi_B12.block(3, 3, 3, 3);

        /// Jacobian of L1_T_Lmplusi wrt ba_hat
        Matrix63 H_L1TLmplusi_ba_hat;
        H_L1TLmplusi_ba_hat.block(0, 0, 3, 3) = H_L1RLmplusi_pB12;
        H_L1TLmplusi_ba_hat.block(3, 0, 3, 3) = H_L1pLmplusi_pB12*H_delP_bias_accel;

        /// Jacobian of L1_T_Lmplusi wrt B11
        Matrix66 H_L1TLmplusi_B11 = H_14*H_12*H_10*H_8*H_7;
        Matrix33 H_L1RLmplusi_pB11 = H_L1TLmplusi_B11.block(0, 3, 3, 3);
        Matrix33 H_L1pLmplusi_pB11 = H_L1TLmplusi_B11.block(3, 3, 3, 3);
        /// Jacobian 1 of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat1;
        H_L1TLmplusi_bg_hat1.block(0, 0, 3, 3) = H_L1RLmplusi_pB11;
        H_L1TLmplusi_bg_hat1.block(3, 0, 3, 3) = H_L1pLmplusi_pB11*H_delP_bias_omega;

        /// Jacobian of L1_T_Lmplusi wrt B3
        Matrix66 H_L1TLmplusi_B3 = H_14*H_13;
        Matrix63 H_L1TLmplusi_RB3 = H_L1TLmplusi_B3.block(0, 0, 6, 3);
        /// Jacobian 2 of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat2;
        H_L1TLmplusi_bg_hat2 = H_L1TLmplusi_RB3*H_exp_Hbg*H_delR_bias_omega;

        /// Jacobian of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat = H_L1TLmplusi_bg_hat1 + H_L1TLmplusi_bg_hat2;

        /// Jacobian of L1_T_Lmplusi wrt b = [ba, bg]
        Matrix6 H_L1TLmplusi_b;
        H_L1TLmplusi_b.block(0, 0, 6, 3) = H_L1TLmplusi_ba_hat;
        H_L1TLmplusi_b.block(0, 3, 6, 3) = H_L1TLmplusi_bg_hat;

        /// Jacobian of L1_T_Lmplusi wrt B2
        Matrix66 H_L1TLmplusi_B2 = H_14*H_12*H_11;
        /// Jacobian of L1_T_Lmplusi wrt delR
        Matrix63 H_L1TLmplusi_delR = H_L1TLmplusi_B2.block(0, 0, 6, 3);
        /// Jacobian of L1_T_Lmplusi wrt delP
        Matrix63 H_L1TLmplusi_delP = H_L1TLmplusi_B2.block(0, 3, 6, 3);

        /// Transform lidar point measurement
        Matrix36 H_xL1_L1TLmplusi; /// Jacobian of x_L1 wrt L1_T_Lmplusi (3 x 6)
        Matrix33 H_xL1_lidar_point_measurement; /// Jacobian of x_L1 wrt L1_T_Lmplusi (3 x 6)
        Point3 x_L1 = L1_T_Lmplusi.transformFrom(lidar_point_measurement, H_xL1_L1TLmplusi, H_xL1_lidar_point_measurement);
        deskewed_points_csv_writer << scan_ts << ", " << point_ts << ", " << scan_id << ", " << plane_id << ", "
                                   << x_L1.x() << ", " << x_L1.y() << ", " << x_L1.z() << std::endl;
    }

    void BatchOptimizer::GetMeasurementCovariance(const gtsam::Pose3 I1_T_Ik, const gtsam::Vector3 I1_v_Ik,
                                                  const imuBias::ConstantBias& Bm, const gtsam::Pose3 Tc,
                                                  const gtsam::Vector4 plane_param_measurement, const gtsam::Point3 lidar_point_measurement,
                                                  const gtsam::Matrix66 P_I1TIk,
                                                  const gtsam::Matrix33 P_I1vIk,
                                                  const gtsam::Matrix66 P_bIk,
                                                  const gtsam::Matrix66 P_Tc,
                                                  double& measurement_covariance) {
        /// Pre-integrated measurements
        Rot3 deltaR = imu_integrator_planar_constraint_->deltaRij();
        Vector3 deltaP = imu_integrator_planar_constraint_->deltaPij();
        Vector3 deltaV = imu_integrator_planar_constraint_->deltaVij();
        double deltaT = imu_integrator_planar_constraint_->deltaTij();
        Vector3 gravity = imu_integrator_planar_constraint_->params()->n_gravity;
        Matrix93 H_bias_omega = imu_integrator_planar_constraint_->preintegrated_H_biasOmega();
        Matrix93 H_bias_accel = imu_integrator_planar_constraint_->preintegrated_H_biasAcc();

        Matrix33 H_delR_bias_accel = H_bias_accel.block(0, 0, 3, 3);
        Matrix33 H_delP_bias_accel = H_bias_accel.block(3, 0, 3, 3);
        Matrix33 H_delV_bias_accel = H_bias_accel.block(6, 0, 3, 3);
        Vector3 bias_accel_hat = Bm.accelerometer();
        Matrix33 H_delR_bias_omega = H_bias_omega.block(0, 0, 3, 3);
        Matrix33 H_delP_bias_omega = H_bias_omega.block(3, 0, 3, 3);
        Matrix33 H_delV_bias_omega = H_bias_omega.block(6, 0, 3, 3);
        Vector3 bias_omega_hat = Bm.gyroscope();


        Pose3 A = Pose3(Rot3::identity(), I1_v_Ik*deltaT + 0.5*gravity*deltaT*deltaT);
        Pose3 B11 = Pose3(Rot3::identity(), H_delP_bias_omega*bias_omega_hat);
        Pose3 B12 = Pose3(Rot3::identity(), H_delP_bias_accel*bias_accel_hat);
        Pose3 B2 = Pose3(deltaR, deltaP);
        Matrix3 H_exp_Hbg;
        Pose3 B3 = Pose3(gtsam::Rot3::Expmap(H_delR_bias_omega*bias_omega_hat, H_exp_Hbg), gtsam::Vector3::Zero());
        Matrix H_1, H_2, H_3, H_4, H_5, H_6, H_7, H_8, H_9, H_10, H_11, H_12, H_13, H_14, H_15;
        Pose3 L1_T_Lmplusi = Tc.inverse(H_1).compose(A, H_2, H_3).compose(I1_T_Ik, H_4, H_5).compose(B11, H_6, H_7).
                compose(B12, H_8, H_9).compose(B2, H_10, H_11).compose(B3, H_12, H_13).compose(Tc, H_14, H_15);

        /// Jacobian of L1_T_Lmplusi wrt I1_T_Ik
        Matrix66 H_L1TLmplusi_I1TIk = H_14*H_12*H_10*H_8*H_6*H_5;

        /// Jacobian of L1_T_Lmplusi wrt A
        Matrix66 H_L1Tmplusi_A = H_14*H_12*H_10*H_8*H_6*H_4*H_3;
        Matrix33 H_L1Rmplusi_pa = H_L1Tmplusi_A.block(0, 3, 3, 3);
        Matrix33 H_L1pmplusi_pa = H_L1Tmplusi_A.block(3, 3, 3, 3);

        /// Jacobian of L1_T_Lmplusi wrt Tc
        Matrix6 H_L1TLmplusi_Tc;
        H_L1TLmplusi_Tc = H_15 + H_14*H_12*H_10*H_8*H_6*H_4*H_2*H_1;

        /// Jacobian of L1_T_Lmplusi wrt I1_v_Ik
        Matrix63 H_L1TLmplusi_I1vIk; ;
        H_L1TLmplusi_I1vIk.block(0, 0, 3, 3) = H_L1Rmplusi_pa;
        H_L1TLmplusi_I1vIk.block(3, 0, 3, 3) = H_L1pmplusi_pa*deltaT;

        /// Jacobian of L1_T_Lmplusi wrt B12
        Matrix66 H_L1TLmplusi_B12 = H_14*H_12*H_10*H_9;
        Matrix33 H_L1RLmplusi_pB12 = H_L1TLmplusi_B12.block(0, 3, 3, 3);
        Matrix33 H_L1pLmplusi_pB12 = H_L1TLmplusi_B12.block(3, 3, 3, 3);

        /// Jacobian of L1_T_Lmplusi wrt ba_hat
        Matrix63 H_L1TLmplusi_ba_hat;
        H_L1TLmplusi_ba_hat.block(0, 0, 3, 3) = H_L1RLmplusi_pB12;
        H_L1TLmplusi_ba_hat.block(3, 0, 3, 3) = H_L1pLmplusi_pB12*H_delP_bias_accel;

        /// Jacobian of L1_T_Lmplusi wrt B11
        Matrix66 H_L1TLmplusi_B11 = H_14*H_12*H_10*H_8*H_7;
        Matrix33 H_L1RLmplusi_pB11 = H_L1TLmplusi_B11.block(0, 3, 3, 3);
        Matrix33 H_L1pLmplusi_pB11 = H_L1TLmplusi_B11.block(3, 3, 3, 3);
        /// Jacobian 1 of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat1;
        H_L1TLmplusi_bg_hat1.block(0, 0, 3, 3) = H_L1RLmplusi_pB11;
        H_L1TLmplusi_bg_hat1.block(3, 0, 3, 3) = H_L1pLmplusi_pB11*H_delP_bias_omega;

        /// Jacobian of L1_T_Lmplusi wrt B3
        Matrix66 H_L1TLmplusi_B3 = H_14*H_13;
        Matrix63 H_L1TLmplusi_RB3 = H_L1TLmplusi_B3.block(0, 0, 6, 3);
        /// Jacobian 2 of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat2;
        H_L1TLmplusi_bg_hat2 = H_L1TLmplusi_RB3*H_exp_Hbg*H_delR_bias_omega;

        /// Jacobian of L1_T_Lmplusi wrt bg_hat
        Matrix63 H_L1TLmplusi_bg_hat = H_L1TLmplusi_bg_hat1 + H_L1TLmplusi_bg_hat2;

        /// Jacobian of L1_T_Lmplusi wrt b = [ba, bg]
        Matrix6 H_L1TLmplusi_b;
        H_L1TLmplusi_b.block(0, 0, 6, 3) = H_L1TLmplusi_ba_hat;
        H_L1TLmplusi_b.block(0, 3, 6, 3) = H_L1TLmplusi_bg_hat;

        /// Jacobian of L1_T_Lmplusi wrt B2
        Matrix66 H_L1TLmplusi_B2 = H_14*H_12*H_11;
        /// Jacobian of L1_T_Lmplusi wrt delR
        Matrix63 H_L1TLmplusi_delR = H_L1TLmplusi_B2.block(0, 0, 6, 3);
        /// Jacobian of L1_T_Lmplusi wrt delP
        Matrix63 H_L1TLmplusi_delP = H_L1TLmplusi_B2.block(0, 3, 6, 3);

        /// Transform lidar point measurement
        Matrix36 H_xL1_L1TLmplusi; /// Jacobian of x_L1 wrt L1_T_Lmplusi (3 x 6)
        Matrix33 H_xL1_lidar_point_measurement; /// Jacobian of x_L1 wrt L1_T_Lmplusi (3 x 6)
        Point3 x_L1 = L1_T_Lmplusi.transformFrom(lidar_point_measurement, H_xL1_L1TLmplusi, H_xL1_lidar_point_measurement);

        /// Point to plane constraint
        Point3 n_L1 = Point3(plane_param_measurement.x(), plane_param_measurement.y(), plane_param_measurement.z());
        double d_L1 = plane_param_measurement.w();

        /// Residual
        Vector1 res = Vector1((n_L1.transpose()*x_L1 + d_L1));

        Matrix13 H_res_xL1 = n_L1.transpose();

        /// Jacobian of res wrt states
        Matrix16 H_res_I1TIk = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_I1TIk;
        Matrix13 H_res_I1vIk = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_I1vIk;
        Matrix16 H_res_b = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_b;
        Matrix16 H_res_Tc = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_Tc;

        /// Jacobian of res wrt measurements
        Matrix13 H_res_lidar_point_measurement = H_res_xL1*H_xL1_lidar_point_measurement;
        Matrix14 H_res_planar_params =  (Matrix14() << x_L1.x(), x_L1.y(), x_L1.z(), 1).finished();
        Matrix13 H_res_delR = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_delR;
        Matrix13 H_res_delP = H_res_xL1*H_xL1_L1TLmplusi*H_L1TLmplusi_delP;

        /// Measurement Covariances
        double sigma_lidar_point_measurement = 1;
        Matrix33 R_lidar_point_measurement = (Matrix33() << sigma_lidar_point_measurement*sigma_lidar_point_measurement,    0,    0,
                                                               0, sigma_lidar_point_measurement*sigma_lidar_point_measurement,    0,
                                                               0,    0, sigma_lidar_point_measurement*sigma_lidar_point_measurement).finished();
        Matrix cov_preintegrated_measurement = imu_integrator_planar_constraint_->preintMeasCov();
        Matrix33 cov_preintegrated_rotation = cov_preintegrated_measurement.block(0, 0, 3, 3);
        Matrix33 cov_preintegrated_position = cov_preintegrated_measurement.block(3, 3, 3, 3);
        Matrix33 cov_preintegrated_velocity = cov_preintegrated_measurement.block(6, 6, 3, 3);

        gtsam::Matrix R = H_res_I1TIk*P_I1TIk*H_res_I1TIk.transpose()
                        + H_res_I1vIk*P_I1vIk*H_res_I1vIk.transpose()
                        + H_res_b*P_bIk*H_res_b.transpose()
                        + H_res_Tc*P_Tc*H_res_Tc.transpose()
                        + H_res_lidar_point_measurement*R_lidar_point_measurement*H_res_lidar_point_measurement.transpose()
//                        + H_res_planar_params*H_res_planar_params.transpose()
                        + H_res_delR*cov_preintegrated_rotation*H_res_delR.transpose()
                        + H_res_delP*cov_preintegrated_position*H_res_delP.transpose();


        measurement_covariance = R(0, 0);
    }

    void BatchOptimizer::PreIntegrateIMUMeasurements(const Eigen::aligned_vector<ImuMeasurement>& imu_measurements,
                                                     gtsam::PreintegratedImuMeasurements& pre_integrated_imu_measurements) {

        for(int i = 1; i < imu_measurements.size(); i++) {
            double dt = (double(imu_measurements[i].timestamp)- double(imu_measurements[i-1].timestamp))/1e9;

            gtsam::Vector3 omega1 = gtsam::Vector3(imu_measurements[i-1].wm);
            gtsam::Vector3 omega2 = gtsam::Vector3(imu_measurements[i].wm);

            gtsam::Vector3 accel1 = gtsam::Vector3(imu_measurements[i-1].am);
            gtsam::Vector3 accel2 = gtsam::Vector3(imu_measurements[i].am);

            pre_integrated_imu_measurements.integrateMeasurement(0.5*(accel1+accel2), 0.5*(omega1+omega2), dt);
        }
    }

    void BatchOptimizer::GetIMUMeasurementBetweenNodes(const uint64_t time0, const uint64_t time1,
                                                       Eigen::aligned_vector<ImuMeasurement>& imu_measurements) {
        imu_measurements.clear(); /// Just making sure, not necessary
        int i;
        for(i = 0; i < imu_measurements_.size(); i++) {
            if(double(time0) < double(imu_measurements_[i].timestamp)) {
                break;
            }
        }
        int j = 0;
        for(j = i; j < imu_measurements_.size(); j++) {
            if(double(time1) < double(imu_measurements_[j].timestamp)) {
                break;
            }
            /// This is after break to ensure that the current timestep is not included
            imu_measurements.emplace_back(imu_measurements_[j]);
        }
//        std::cout << "index i: " << i << "\t index j: " << j << std::endl;
    }

    bool BatchOptimizer::FailureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            std::cout << "Velocity: " << vel.transpose() << std::endl;
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            std::cout << "ba: " << ba.transpose() << std::endl;
            std::cout << "bg: " << bg.transpose() << std::endl;
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }
        return false;
    }

    void BatchOptimizer::AddTrajConstraints() {
        std::cout << "Started adding pose constraints" << std::endl;
        bool system_initialized = false;
        bool is_first_pose = true;
        uint64_t prev_timestamp;
        gtsam::Pose3 first_pose;
        std::cout << "imu_trajectory_size: " << imu_trajectory_.size() << std::endl;
        gtsam::Pose3 prev_pose;
        for (auto &value : imu_trajectory_) {
            PoseVel curr_pose_vel = value.second;
            uint64_t curr_timestamp = curr_pose_vel.timestamp;
            Eigen::Quaterniond curr_quaternion_eig = curr_pose_vel.orientation;
            Eigen::Matrix3d curr_rotation_eig = curr_quaternion_eig.toRotationMatrix();
            gtsam::Rot3 curr_orientation_gtsam = gtsam::Rot3(curr_rotation_eig);
            Eigen::Vector3d curr_position_eig = curr_pose_vel.position;
            gtsam::Vector3 curr_position_gtsam = gtsam::Vector3(curr_position_eig.x(), curr_position_eig.y(), curr_position_eig.z());
            gtsam::Pose3 curr_pose = gtsam::Pose3(curr_orientation_gtsam, curr_position_gtsam);
            if (!system_initialized) {
                /// Initial Pose
                prev_pose_ = curr_pose;
                gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor(X(0), prev_pose_, prior_pose_noise_);
                graph_factors_.add(prior_pose_factor);

                /// Initial Velocity
                prev_vel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> prior_velocity_factor(V(0), prev_vel_, prior_vel_noise_);
                graph_factors_.add(prior_velocity_factor);

                /// Initial Bias
                prev_bias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias_factor(B(0), prev_bias_, prior_bias_noise_);
                graph_factors_.add(prior_bias_factor);

                /// Add values
                graph_values_.insert(X(0), prev_pose_);
                graph_values_.insert(V(0), prev_vel_);
                graph_values_.insert(B(0), prev_bias_);

                /// Optimize
//                LevenbergMarquardtOptimizer optimizer(graph_factors_, graph_values_);
//                optimization_result = optimizer.optimize();

                /// Overwrite the beginning of preintegration for the next step
                prev_pose_ = graph_values_.at<gtsam::Pose3>(X(key));
                if(!pose_vector_.insert(std::make_pair(size_t(key), prev_pose_)).second)
                    std::cout << "Could not insert at: " << key << std::endl;


                prev_vel_ = graph_values_.at<gtsam::Vector3>(V(key));
                if(!velocity_vector_.insert(std::make_pair(size_t(key), prev_vel_)).second)
                    std::cout << "Could not insert at: " << key << std::endl;

                prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);

                prev_bias_ = graph_values_.at<gtsam::imuBias::ConstantBias>(B(key));
                if(!bias_vector_.insert(std::make_pair(size_t(key), prev_bias_)).second)
                    std::cout << "Could not insert at: " << key << std::endl;

                if(FailureDetection(prev_vel_, prev_bias_)) {
                    ROS_ERROR_STREAM("Large jump at key: " << key);
                }
//                Eigen::Quaterniond prev_pose_quat_eig = Eigen::Quaterniond(prev_pose_.rotation().matrix());
//                imu_trajectory_out_csv_writer << imu_trajectory_.at(key).timestamp << ", "
//                                              << prev_pose_quat_eig.x() << ","
//                                              << prev_pose_quat_eig.y() << ","
//                                              << prev_pose_quat_eig.z() << ","
//                                              << prev_pose_quat_eig.w() << ","
//                                              << prev_pose_.translation().x() << ","
//                                              << prev_pose_.translation().y() << ","
//                                              << prev_pose_.translation().z() << std::endl;

                imu_integrator_->resetIntegrationAndSetBias(prev_bias_);
                prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
                key = 1;
                system_initialized = true;
                prev_timestamp = curr_timestamp;
                prev_pose = curr_pose;
                continue;
            }

            Eigen::aligned_vector<ImuMeasurement> imu_measurements;
            GetIMUMeasurementBetweenNodes(prev_timestamp, curr_timestamp, imu_measurements);
            PreIntegrateIMUMeasurements(imu_measurements, *imu_integrator_);

            /// Add Imu factor
            const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_);
            gtsam::ImuFactor imu_factor(X(key-1), V(key-1), X(key), V(key),B(key-1), preint_imu);
            graph_factors_.add(imu_factor);

            /// Add imu bias between factor
            graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key-1), B(key), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_->deltaTij()) * noise_model_between_bias_)));

//            /// Add pose between factor
//            gtsam::Pose3 imu_odometry = prev_pose.between(curr_pose);
//            gtsam::BetweenFactor<gtsam::Pose3> pose_between_factor(X(key-1), X(key), imu_odometry, correction_noise_imu_poseF2F_);
//            graph_factors_.add(pose_between_factor);

            /// Add prior absolute Pose factor
            gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor(X(key), curr_pose, correction_noise_imu_poseF2M_);
            graph_factors_.add(prior_pose_factor);

            /// Insert predicted values
            gtsam::NavState prop_state = imu_integrator_->predict(prev_state_, prev_bias_);
            graph_values_.insert(X(key), prop_state.pose());
            graph_values_.insert(V(key), prop_state.v());
            graph_values_.insert(B(key), prev_bias_);

            /// Optimizer
//            LevenbergMarquardtOptimizer optimizer(graph_factors_, graph_values_);
//            optimization_result = optimizer.optimize();

            /// Overwrite the beginning of preintegration for the next step
            prev_pose_ = graph_values_.at<gtsam::Pose3>(X(key));
            if(!pose_vector_.insert(std::make_pair(size_t(key), prev_pose_)).second)
                std::cout << "Could not insert at: " << key << std::endl;

            prev_vel_ = graph_values_.at<gtsam::Vector3>(V(key));
            if(!velocity_vector_.insert(std::make_pair(size_t(key), prev_vel_)).second)
                std::cout << "Could not insert at: " << key << std::endl;

            prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);

            prev_bias_ = graph_values_.at<gtsam::imuBias::ConstantBias>(B(key));
            if(!bias_vector_.insert(std::make_pair(size_t(key), prev_bias_)).second)
                std::cout << "Could not insert at: " << key << std::endl;

            if(FailureDetection(prev_vel_, prev_bias_)) {
                ROS_ERROR_STREAM("Large jump at key: " << key);
            }
//            Eigen::Quaterniond prev_pose_quat_eig = Eigen::Quaterniond(prev_pose_.rotation().matrix());
//            imu_trajectory_out_csv_writer << imu_trajectory_.at(key).timestamp << ", "
//                                          << prev_pose_quat_eig.x() << ","
//                                          << prev_pose_quat_eig.y() << ","
//                                          << prev_pose_quat_eig.z() << ","
//                                          << prev_pose_quat_eig.w() << ","
//                                          << prev_pose_.translation().x() << ","
//                                          << prev_pose_.translation().y() << ","
//                                          << prev_pose_.translation().z() << std::endl;
            imu_integrator_->resetIntegrationAndSetBias(prev_bias_);
            prev_timestamp = curr_timestamp;
            prev_pose = curr_pose;
            key++;
        }
        std::cout << "Pose vector size: " << pose_vector_.size() << std::endl;
        std::cout << "Velocity vector size: " << velocity_vector_.size() << std::endl;
        std::cout << "Bias vector size: " << bias_vector_.size() << std::endl;
        std::cout << "Done adding pose constraints" << std::endl;
    }

    void BatchOptimizer::AddPlaneConstraints() {
        std::cout << "Started adding planar constraints" << std::endl;
        Eigen::aligned_vector<double> no_of_points_per_plane_id;
        int no_of_nonzero_plane_ids = 0;
        for(int i = 0; i < plane_params_.size(); i++) {
            int counter = 0;
            for(int j = 0; j < planar_points_.size(); j++) {
                if(i == planar_points_.at(j).plane_id)
                    counter++;
            }
            if(counter > 0)
                no_of_nonzero_plane_ids++;
            no_of_points_per_plane_id.push_back(counter);
        }

        size_t prev_scan_id = 0;
        size_t curr_scan_id = 0;
        gtsam::Pose3 prev_pose;
        uint64_t prev_prev_scan_timestamp;
        uint64_t prev_scan_timestamp;
        uint64_t curr_scan_timestamp;
        gtsam::Pose3 curr_pose;
        /// Covariance containers
        gtsam::Matrix66 pose_cov = gtsam::Matrix66::Identity();
        gtsam::Matrix33 velocity_cov = gtsam::Matrix33::Identity();
        gtsam::Matrix66 bias_cov = gtsam::Matrix66::Identity();
        gtsam::Matrix66 calib_cov = gtsam::Matrix66::Identity();

        int no_of_planar_factors = 0;
        for(auto &value : planar_points_) {

            curr_scan_timestamp = value.timestamp_scan;
            uint64_t curr_point_timestamp = value.timestamp_point;
            curr_scan_id = value.scan_id;
            size_t curr_plane_id = value.plane_id;
            PoseVel curr_pose_vel = imu_trajectory_.at(curr_scan_id);

            Eigen::Quaterniond curr_quaternion_eig = curr_pose_vel.orientation;
            Eigen::Matrix3d curr_rotation_eig = curr_quaternion_eig.toRotationMatrix();
            gtsam::Rot3 curr_orientation_gtsam = gtsam::Rot3(curr_rotation_eig);
            Eigen::Vector3d curr_position_eig = curr_pose_vel.position;
            gtsam::Vector3 curr_position_gtsam = gtsam::Vector3(curr_position_eig.x(), curr_position_eig.y(), curr_position_eig.z());
            curr_pose = gtsam::Pose3(curr_orientation_gtsam, curr_position_gtsam);

            if(curr_scan_id != prev_scan_id) {
                if(prev_scan_id==0) {
                    std::cout << "At scan id: " << prev_scan_id << std::endl;
                    /// Initial Pose
                    prev_pose_ = curr_pose;
                    gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor(X(prev_scan_id), prev_pose_, prior_pose_noise_);
                    graph_factors_.add(prior_pose_factor);

                    /// Initial Velocity
                    prev_vel_ = gtsam::Vector3(0, 0, 0);
                    gtsam::PriorFactor<gtsam::Vector3> prior_velocity_factor(V(prev_scan_id), prev_vel_, prior_vel_noise_);
                    graph_factors_.add(prior_velocity_factor);

                    /// Initial Bias
                    prev_bias_ = gtsam::imuBias::ConstantBias();
                    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias_factor(B(prev_scan_id), prev_bias_, prior_bias_noise_);
                    graph_factors_.add(prior_bias_factor);

                    /// Initial Calib
                    gtsam::PriorFactor<gtsam::Pose3> prior_calib_factor(C(0), prev_calib_pose_, prior_calib_noise_);
                    graph_factors_.add(prior_calib_factor);

                    /// Add values
                    graph_values_.insert(X(prev_scan_id), prev_pose_);
                    graph_values_.insert(V(prev_scan_id), prev_vel_);
                    graph_values_.insert(B(prev_scan_id), prev_bias_);
                    graph_values_.insert(C(0), prev_calib_pose_);

                    /// Optimize
                    gtsam::LevenbergMarquardtParams lm_params;
                    lm_params.setVerbosityLM("SUMMARY");
                    LevenbergMarquardtOptimizer optimizer(graph_factors_, graph_values_, lm_params);
                    optimization_result = optimizer.optimize();
//                    isam2_optimizer.update(graph_factors_, graph_values_);
//                    isam2_optimizer.update();
//                    graph_factors_.resize(0);
//                    graph_values_.clear();
//                    optimization_result = isam2_optimizer.calculateEstimate();

                    /// Overwrite the beginning of preintegration for the next step
                    prev_pose_ = optimization_result.at<gtsam::Pose3>(X(prev_scan_id));
                    prev_vel_ = optimization_result.at<gtsam::Vector3>(V(prev_scan_id));
                    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
                    prev_bias_ = optimization_result.at<gtsam::imuBias::ConstantBias>(B(prev_scan_id));

                    if(FailureDetection(prev_vel_, prev_bias_)) {
                        ROS_ERROR_STREAM("Large jump at key: " << prev_scan_id);
                    }

                    imu_velocity_out_csv_writer << prev_vel_.x() << ", "<< prev_vel_.y() << ", "<< prev_vel_.z() << std::endl;

                    imu_integrator_->resetIntegrationAndSetBias(prev_bias_);
                    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
                    prev_calib_pose_ = optimization_result.at<gtsam::Pose3>(C(0));

                    init_calib_csv_writer << imu_T_lidar.translation().x() << ", " << imu_T_lidar.translation().y() << ", "
                                          << imu_T_lidar.translation().z() << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;
                    final_calib_csv_writer << prev_calib_pose_.translation().x() << ", " << prev_calib_pose_.translation().y() << ", "
                                           << prev_calib_pose_.translation().z() << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;

                    /// get covariances
                    gtsam::Marginals marginals(graph_factors_, optimization_result);
                    pose_cov = marginals.marginalCovariance(X(prev_scan_id));
                    velocity_cov = marginals.marginalCovariance(V(prev_scan_id));
                    bias_cov = marginals.marginalCovariance(B(prev_scan_id));
                    calib_cov = marginals.marginalCovariance(C(0));

//                    pose_cov = isam2_optimizer.marginalCovariance(X(prev_scan_id));
//                    velocity_cov = isam2_optimizer.marginalCovariance(V(prev_scan_id));
//                    bias_cov = isam2_optimizer.marginalCovariance(B(prev_scan_id));
//                    calib_cov = isam2_optimizer.marginalCovariance(C(0));
                } else {
                    std::cout << "At scan id: " << prev_scan_id << std::endl;
                    Eigen::aligned_vector<ImuMeasurement> imu_measurements;
                    GetIMUMeasurementBetweenNodes(prev_prev_scan_timestamp, prev_scan_timestamp, imu_measurements);
                    PreIntegrateIMUMeasurements(imu_measurements, *imu_integrator_);

                    /// Add Imu factor
                    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_);
                    gtsam::ImuFactor imu_factor(X(prev_scan_id-1), V(prev_scan_id-1), X(prev_scan_id), V(prev_scan_id),B(prev_scan_id-1), preint_imu);
                    graph_factors_.add(imu_factor);

                    /// Add imu bias between factor
                    graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(prev_scan_id-1),
                                                                                                               B(prev_scan_id),
                                                                                                               gtsam::imuBias::ConstantBias(),
                                                                                                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_->deltaTij()) * noise_model_between_bias_)));

                    /// Add prior pose factor
                    gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor(X(prev_scan_id), curr_pose, correction_noise_imu_poseF2M_);
                    graph_factors_.add(prior_pose_factor);

                    /// Insert predicted values
                    gtsam::NavState prop_state = imu_integrator_->predict(prev_state_, prev_bias_);
                    graph_values_.insert(X(prev_scan_id), prop_state.pose());
                    graph_values_.insert(V(prev_scan_id), prop_state.v());
                    graph_values_.insert(B(prev_scan_id), prev_bias_);

                    /// Optimizer
                    gtsam::LevenbergMarquardtParams lm_params;
                    lm_params.setVerbosityLM("SUMMARY");
                    LevenbergMarquardtOptimizer optimizer(graph_factors_, graph_values_, lm_params);
                    optimization_result = optimizer.optimize();
//                    isam2_optimizer.update(graph_factors_, graph_values_);
//                    isam2_optimizer.update();
//                    graph_factors_.resize(0);
//                    graph_values_.clear();
//                    optimization_result = isam2_optimizer.calculateEstimate();

                    /// Overwrite the beginning of preintegration for the next step
                    prev_pose_ = optimization_result.at<gtsam::Pose3>(X(prev_scan_id));
                    prev_vel_ = optimization_result.at<gtsam::Vector3>(V(prev_scan_id));
                    prev_bias_ = optimization_result.at<gtsam::imuBias::ConstantBias>(B(prev_scan_id));

                    if(FailureDetection(prev_vel_, prev_bias_)) {
                        ROS_ERROR_STREAM("Large jump at key: " << prev_scan_id);
                    }

                    imu_integrator_->resetIntegrationAndSetBias(prev_bias_);

                    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
                    prev_calib_pose_ = optimization_result.at<gtsam::Pose3>(C(0));

                    init_calib_csv_writer << imu_T_lidar.translation().x() << ", " << imu_T_lidar.translation().y() << ", "
                                          << imu_T_lidar.translation().z() << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                                          << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;
                    final_calib_csv_writer << prev_calib_pose_.translation().x() << ", " << prev_calib_pose_.translation().y() << ", "
                                           << prev_calib_pose_.translation().z() << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                                           << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;

                    /// get covariances
                    gtsam::Marginals marginals(graph_factors_, optimization_result);
                    pose_cov = marginals.marginalCovariance(X(prev_scan_id));
                    velocity_cov = marginals.marginalCovariance(V(prev_scan_id));
                    bias_cov = marginals.marginalCovariance(B(prev_scan_id));
                    calib_cov = marginals.marginalCovariance(C(0));
//                    pose_cov = isam2_optimizer.marginalCovariance(X(prev_scan_id));
//                    velocity_cov = isam2_optimizer.marginalCovariance(V(prev_scan_id));
//                    bias_cov = isam2_optimizer.marginalCovariance(B(prev_scan_id));
//                    calib_cov = isam2_optimizer.marginalCovariance(C(0));
                }
                prev_prev_scan_timestamp = prev_scan_timestamp;
            }

            if(no_of_points_per_plane_id.at(curr_plane_id) == 0) {
                ROS_ERROR_STREAM("no_of_points_per_plane_id.at(plane_id) == 0");
                ros::shutdown();
            }

//            if(curr_scan_id != 0) {
                double weight = 1;
                gtsam::Point3 lidar_point_measurement = gtsam::Point3(value.point.x(), value.point.y(), value.point.z());
                gtsam::Vector4 plane_param_measurement = gtsam::Vector4(plane_params_.at(curr_plane_id).x(),plane_params_.at(curr_plane_id).y(),
                                                                        plane_params_.at(curr_plane_id).z(),plane_params_.at(curr_plane_id).w());
                Eigen::aligned_vector<ImuMeasurement> imu_measurements;
                GetIMUMeasurementBetweenNodes(curr_scan_timestamp, curr_scan_timestamp+curr_point_timestamp, imu_measurements);

                PreIntegrateIMUMeasurements(imu_measurements, *imu_integrator_planar_constraint_);

                gtsam::Rot3 deltaR = imu_integrator_planar_constraint_->deltaRij();
                gtsam::Vector3 deltaP = imu_integrator_planar_constraint_->deltaPij();
                gtsam::Vector3 deltaV = imu_integrator_planar_constraint_->deltaVij();
                double deltaT = imu_integrator_planar_constraint_->deltaTij();
                gtsam::Vector3 gravity_vector = imu_integrator_planar_constraint_->params()->n_gravity;

                PreIntegratedIMUMeasurements preintegrated_imu_measurements;
                preintegrated_imu_measurements.deltaR = deltaR;
                preintegrated_imu_measurements.deltaP = deltaP;
                preintegrated_imu_measurements.deltaV = deltaV;
                preintegrated_imu_measurements.deltaT = deltaT;
                preintegrated_imu_measurements.gravity = gravity_vector;
                preintegrated_imu_measurements.H_bias_accel = imu_integrator_planar_constraint_->preintegrated_H_biasAcc();
                preintegrated_imu_measurements.H_bias_omega = imu_integrator_planar_constraint_->preintegrated_H_biasOmega();

                imu_integrator_planar_constraint_->resetIntegrationAndSetBias(prev_bias_);

                double measurement_covariance;
                GetMeasurementCovariance(curr_pose, prev_vel_, prev_bias_, prev_calib_pose_, plane_param_measurement,
                                         lidar_point_measurement, pose_cov, velocity_cov, bias_cov, calib_cov,
                                         measurement_covariance);
//                std::cout << "measurement_covariance: " << measurement_covariance << std::endl;
//                correction_noise_planar_constraint_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) <<
//                                                                                                            sqrt(double(no_of_nonzero_plane_ids)*no_of_points_per_plane_id.at(curr_plane_id))).finished());

                std::cout << "measurement noise: " << measurement_covariance << std::endl;
//                correction_noise_planar_constraint_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) <<
//                                                                                                        sqrt(double(no_of_nonzero_plane_ids)*no_of_points_per_plane_id.at(curr_plane_id))*measurement_covariance).finished());
                correction_noise_planar_constraint_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) <<
                                                                                                            sqrt(measurement_covariance)).finished());
                graph_factors_.add(boost::make_shared<PointToPlaneFactor3>(X(curr_scan_id), V(curr_scan_id),
                                                                                  B(curr_scan_id),C(0), preintegrated_imu_measurements,
                                                                                  plane_param_measurement, lidar_point_measurement, 1,
                                                                                  correction_noise_planar_constraint_));

//            }
            prev_scan_id = curr_scan_id;
            prev_scan_timestamp = curr_scan_timestamp;
        }

        std::cout << "At scan id: " << prev_scan_id << std::endl;
        Eigen::aligned_vector<ImuMeasurement> imu_measurements;
        GetIMUMeasurementBetweenNodes(prev_prev_scan_timestamp, prev_scan_timestamp, imu_measurements);
        PreIntegrateIMUMeasurements(imu_measurements, *imu_integrator_);

        /// Add Imu factor
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imu_integrator_);
        gtsam::ImuFactor imu_factor(X(prev_scan_id-1), V(prev_scan_id-1), X(prev_scan_id), V(prev_scan_id),B(prev_scan_id-1), preint_imu);

        graph_factors_.add(imu_factor);

        /// Add imu bias between factor
        graph_factors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(prev_scan_id-1),
                                                                              B(prev_scan_id),
                                                                              gtsam::imuBias::ConstantBias(),
                                                                              gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_->deltaTij()) * noise_model_between_bias_)));

        /// Add prior pose factor
        gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor(X(prev_scan_id), curr_pose, correction_noise_imu_poseF2M_);
        graph_factors_.add(prior_pose_factor);

        /// Insert predicted values
        gtsam::NavState prop_state = imu_integrator_->predict(prev_state_, prev_bias_);
        graph_values_.insert(X(prev_scan_id), prop_state.pose());
        graph_values_.insert(V(prev_scan_id), prop_state.v());
        graph_values_.insert(B(prev_scan_id), prev_bias_);

        /// Optimizer
        gtsam::LevenbergMarquardtParams lm_params;
        lm_params.setVerbosityLM("SUMMARY");
        LevenbergMarquardtOptimizer optimizer(graph_factors_, graph_values_, lm_params);
        optimization_result = optimizer.optimize();
//        isam2_optimizer.update(graph_factors_, graph_values_);
//        isam2_optimizer.update();
//        graph_factors_.resize(0);
//        graph_values_.clear();
//        optimization_result = isam2_optimizer.calculateEstimate();

        /// Overwrite the beginning of preintegration for the next step
        prev_pose_ = optimization_result.at<gtsam::Pose3>(X(prev_scan_id));
        prev_vel_ = optimization_result.at<gtsam::Vector3>(V(prev_scan_id));
        prev_bias_ = optimization_result.at<gtsam::imuBias::ConstantBias>(B(prev_scan_id));

        if(FailureDetection(prev_vel_, prev_bias_)) {
            ROS_ERROR_STREAM("Large jump at key: " << prev_scan_id);
        }

        imu_integrator_->resetIntegrationAndSetBias(prev_bias_);

        prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
        prev_calib_pose_ = optimization_result.at<gtsam::Pose3>(C(0));

        init_calib_csv_writer << imu_T_lidar.translation().x() << ", " << imu_T_lidar.translation().y() << ", "
                              << imu_T_lidar.translation().z() << ", "
                              << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                              << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                              << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;
        final_calib_csv_writer << prev_calib_pose_.translation().x() << ", " << prev_calib_pose_.translation().y() << ", "
                               << prev_calib_pose_.translation().z() << ", "
                               << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).x()*180/M_PI << ", "
                               << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).y()*180/M_PI << ", "
                               << prev_calib_pose_.rotation().matrix().eulerAngles(0, 1, 2).z()*180/M_PI << std::endl;

        std::cout << "Done adding planar constraints" << std::endl;
    }

    void BatchOptimizer::Solve() {
        std::cout << "Optimizing" << std::endl;
        std::cout << "No of factors: " << graph_factors_.size() << std::endl;
        std::cout << "No of variables: " << graph_values_.size() << std::endl;
        gtsam::LevenbergMarquardtParams lm_params;

        lm_params.setVerbosityLM("SUMMARY");
        std::cout << "lm_params: " << lm_params.getVerbosityLM() << std::endl;
        gtsam::LevenbergMarquardtOptimizer lm_optimizer(graph_factors_, graph_values_, lm_params);
        optimization_result = lm_optimizer.optimize();
        std::cout << "Optimized" << std::endl;
    }

    void BatchOptimizer::LogTrajectory() {
        std::cout << "Logging Data" << std::endl;
        for(int i = 0; i < imu_trajectory_.size(); i++) {
            gtsam::Pose3 curr_pose = optimization_result.at<gtsam::Pose3>(X(i));
            gtsam::Vector3 curr_vel = optimization_result.at<gtsam::Vector3>(V(i));
            gtsam::Pose3 calibration_pose = optimization_result.at<gtsam::Pose3>(C(0));

            std::cout << "Key: " << i << std::endl;
            std::cout << "IMU Translation: " << curr_pose.translation().transpose() << std::endl;
            std::cout << "IMU Rotation\n" << std::endl;
            std::cout << curr_pose.rotation().matrix()<< std::endl << std::endl;
            Eigen::Matrix3d curr_rotation_eig = curr_pose.rotation().matrix();
            Eigen::Quaterniond curr_rotation_quat_eig = Eigen::Quaterniond(curr_rotation_eig);
            std::cout << "IMU Velocity: " << curr_vel.transpose() << std::endl << std::endl;
            imu_trajectory_out_csv_writer << imu_trajectory_.at(i).timestamp << ", "
                                          << curr_rotation_quat_eig.x() << ","
                                          << curr_rotation_quat_eig.y() << ","
                                          << curr_rotation_quat_eig.z() << ","
                                          << curr_rotation_quat_eig.w() << ","
                                          << curr_pose.translation().x() << ","
                                          << curr_pose.translation().y() << ","
                                          << curr_pose.translation().z() << ","
                                          << curr_vel.x() << ","
                                          << curr_vel.y() << ","
                                          << curr_vel.z() << "\n";

//            std::cout << "Gyro bias: " << optimization_result.at<gtsam::imuBias::ConstantBias>(B(i)).gyroscope().transpose() << std::endl;
//            std::cout << "Accel bias: " << optimization_result.at<gtsam::imuBias::ConstantBias>(B(i)).accelerometer().transpose() << std::endl << std::endl;
//            gtsam::Pose3 calib = optimization_result.at<gtsam::Pose3>(C(0));
//            std::cout << "Calib Translation Initial: " << imu_T_lidar.translation().transpose() << std::endl;
//            std::cout << "Calib Rotation Initial: " << imu_T_lidar.rotation().matrix().eulerAngles(0, 1, 2).transpose()*180/M_PI << std::endl;
//            std::cout << "Calib Translation Final: " << calib.translation().transpose() << std::endl;
//            std::cout << "Calib Rotation Final: " << calib.rotation().matrix().eulerAngles(0, 1, 2).transpose()*180/M_PI << std::endl;
//            std::cout << "---------------------------------------------------" << std::endl << std::endl;
        }
        std::cout << "Logged" << std::endl;
    }


    void BatchOptimizer::ValidationUsingMap() {
        /// Covariance containers
        gtsam::Matrix66 pose_cov = Matrix66::Identity();
        gtsam::Matrix33 velocity_cov = Matrix33::Identity();
        gtsam::Matrix66 bias_cov = Matrix66::Identity();
        gtsam::Matrix66 calib_cov = Matrix66::Identity();

        std::cout << "Generating A Validation Map" << std::endl;
        for(auto &value : planar_points_) {
            uint64_t curr_scan_timestamp = value.timestamp_scan;
            uint64_t curr_point_timestamp = value.timestamp_point;
            size_t curr_scan_id = value.scan_id;
            size_t curr_plane_id = value.plane_id;

            Pose3 curr_pose = optimization_result.at<Pose3>(X(curr_scan_id));
            Vector3 curr_vel = optimization_result.at<Vector3>(V(curr_scan_id));
            imuBias::ConstantBias curr_bias = optimization_result.at<imuBias::ConstantBias>(B(curr_scan_id));
            Pose3 calib_param = optimization_result.at<Pose3>(C(0));

            double weight = 1;
            gtsam::Point3 lidar_point_measurement = gtsam::Point3(value.point.x(), value.point.y(), value.point.z());
            gtsam::Vector4 plane_param_measurement = gtsam::Vector4(plane_params_.at(curr_plane_id).x(),plane_params_.at(curr_plane_id).y(),
                                                                    plane_params_.at(curr_plane_id).z(),plane_params_.at(curr_plane_id).w());
            Eigen::aligned_vector<ImuMeasurement> imu_measurements;
            GetIMUMeasurementBetweenNodes(curr_scan_timestamp, curr_scan_timestamp+curr_point_timestamp, imu_measurements);

            PreIntegrateIMUMeasurements(imu_measurements, *imu_integrator_planar_constraint_);

            gtsam::Rot3 deltaR = imu_integrator_planar_constraint_->deltaRij();
            gtsam::Vector3 deltaP = imu_integrator_planar_constraint_->deltaPij();
            gtsam::Vector3 deltaV = imu_integrator_planar_constraint_->deltaVij();
            double deltaT = imu_integrator_planar_constraint_->deltaTij();
            gtsam::Vector3 gravity_vector = imu_integrator_planar_constraint_->params()->n_gravity;

            PreIntegratedIMUMeasurements preintegrated_imu_measurements;
            preintegrated_imu_measurements.deltaR = deltaR;
            preintegrated_imu_measurements.deltaP = deltaP;
            preintegrated_imu_measurements.deltaV = deltaV;
            preintegrated_imu_measurements.deltaT = deltaT;
            preintegrated_imu_measurements.gravity = gravity_vector;
            preintegrated_imu_measurements.H_bias_accel = imu_integrator_planar_constraint_->preintegrated_H_biasAcc();
            preintegrated_imu_measurements.H_bias_omega = imu_integrator_planar_constraint_->preintegrated_H_biasOmega();

//            imu_integrator_planar_constraint_->resetIntegrationAndSetBias(prev_bias_);

//            correction_noise_planar_constraint_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << sqrt(1)).finished());

//            PointToPlaneFactor3 pointToPlaneFactor3(X(curr_scan_id), V(curr_scan_id),
//                                                    B(curr_scan_id),C(0), preintegrated_imu_measurements,
//                                                    plane_param_measurement, lidar_point_measurement, 1,
//                                                    correction_noise_planar_constraint_);
//            Matrix16 H_TIk;
//            Matrix13 H_vIk;
//            Matrix16 H_bk;
//            Matrix16 H_Tc;
//            pointToPlaneFactor3.computeErrorAndJacobians(curr_pose, curr_vel, curr_bias, prev_calib_pose_, H_TIk, H_vIk, H_bk, H_Tc);
//            double measurement_covariance;
            GetDeskewedMap(curr_scan_timestamp, curr_point_timestamp, curr_scan_id, curr_plane_id, curr_pose, curr_vel,
                           curr_bias, calib_param, preintegrated_imu_measurements, lidar_point_measurement);
//            gtsam::Matrix11 R = H_TIk*pose_cov*H_TIk.transpose() + H_vIk*velocity_cov*H_vIk.transpose() + H_bk*bias_cov*H_bk.transpose() + H_Tc*calib_cov*H_Tc.transpose();
//            std::cout << "R: " << R.matrix() << std::endl;

        }
        std::cout << "Generated Validation Data" << std::endl;
    }
}