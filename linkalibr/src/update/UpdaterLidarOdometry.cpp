//
// Created by usl on 11/28/20.
//

#include "UpdaterLidarOdometry.h"

using namespace lin_core;
using namespace lin_estimator;

void UpdaterLidarOdometry::updateScan2Scan(State *current_state, relativePose lodom, bool &did_update) {
    /// Li_T_Lj
    Eigen::Matrix4d Li_T_Lj = lodom.odometry_ij;

    /// Odometry time stamps
    /// This has to correspond to the previous state
    /// I assume it does, have to remove this assumption later
    double odom_ts_i = lodom.timestamp_i;
    /// This has to correspond to the current state
    /// I assume it does, have to remove this assumption later
    double odom_ts_j = lodom.timestamp_j;

    /// IMU pose at time stamp i
    Pose *imuPose_i = current_state->_clones_IMU.at(odom_ts_i);
    Eigen::Matrix<double, 3, 3> Ii_R_G = imuPose_i->Rot(); // Ii_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ii = imuPose_i->pos(); // p_Ii in G

    /// IMU pose at time stamp j
    Pose *imuPose_j = current_state->_clones_IMU.at(odom_ts_j);
    Eigen::Matrix<double, 3, 3> Ij_R_G = imuPose_j->Rot(); // Ij_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ij = imuPose_j->pos(); // p_Ij in G
    Eigen::Matrix<double, 3, 3> G_R_Ij = Ij_R_G.transpose(); // G_R_Ij

    /// IMU to LIDAR extrinsic calibration
    Pose *calibration = current_state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_t_L = calibration->pos();
    /// Predicted measurements using best estimates of states
    Eigen::Matrix<double, 3, 3> Li_R_Lj_hat = I_R_L.transpose()*Ii_R_G*G_R_Ij*I_R_L;
    Eigen::Matrix<double, 3, 1> Li_t_Lj_hat = I_R_L.transpose()*( Ii_R_G*G_R_Ij*I_t_L + Ii_R_G*(G_p_Ij-G_p_Ii) - I_t_L );

    /// True measurements Li_R_Lj,  Li_t_Lj
    Eigen::Matrix<double, 3, 3> Li_R_Lj = Li_T_Lj.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> Li_t_Lj = Li_T_Lj.block(0, 3, 3, 1);

    std::vector<Type*> x_order;
    int total_hx = 0;

    /// Add the clone i: IMU pose corresponding prev update time
    x_order.push_back(imuPose_i);
    total_hx += imuPose_i->size();

    /// Add the clone j: IMU pose corresponding curr update time
    x_order.push_back(imuPose_j);
    total_hx += imuPose_j->size();

    if(current_state->_options.do_calib_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration);
        total_hx += calibration->size();
    }

    /// Assign Jacobian blocks
    /// H1 block, corresponding to res1
    Eigen::MatrixXd H1_xi_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xi_trans = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xj_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xj_trans = Eigen::MatrixXd::Zero(3, 3);
    H1_xi_rot = I_R_L.transpose();
    H1_xj_rot = -I_R_L.transpose()*Ii_R_G*Ij_R_G.transpose();

    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(3, total_hx);
    H_x.block(0, 0, 3, 3) = H1_xi_rot;
    H_x.block(0, 3, 3, 3) = H1_xi_trans;
    H_x.block(0, 6, 3, 3) = H1_xj_rot;
    H_x.block(0, 9, 3, 3) = H1_xj_trans;

    Eigen::MatrixXd H1_xc_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xc_trans = Eigen::MatrixXd::Zero(3, 3);
    if(current_state->_options.do_calib_extrinsic) {
        H1_xc_rot = I_R_L.transpose()*(Ii_R_G*Ij_R_G.transpose()-Eigen::MatrixXd::Identity(3, 3));
        H_x.block(0, 12, 3, 3) = H1_xc_rot;
        H_x.block(0, 15, 3, 3) = H1_xc_trans;
    }

    /// Assign measurement residual
    Eigen::Matrix<double, 3, 1> res_rot =  Log_so3(Li_R_Lj*Li_R_Lj_hat.transpose());
    Eigen::MatrixXd R = std::pow(_options.noise_rotation, 2)*Eigen::MatrixXd::Identity(3, 3);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res_rot.dot(S.llt().solve(res_rot));

    /// TODO: I actually need to check if the dimension should be 3 or not, what does the dimension really mean physically?
    boost::math::chi_squared chi_squared_dist(3); // I know res_trans.rows() is 3
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);

    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update] unsuccessful updateScan2Scan \n" RESET);
            did_update = false;
            return;
        }
    }
    /// Update
    StateHelper::EKFUpdate(current_state, x_order, H_x, res_rot, std::pow(_options.noise_rotation, 2) * Eigen::MatrixXd::Identity(3, 3));
    printf(BOLDGREEN "[Update] successful updateScan2Scan \n" RESET);
    did_update = true;
}

void UpdaterLidarOdometry::updateScan2GlobalMap(State *current_state, Eigen::Matrix4d L1_T_Lk, Eigen::Matrix4d G_T_I1, double timestamp, bool& did_update) {
    /// Initial IMU pose
    Eigen::Matrix<double, 3, 3> G_R_I1 = G_T_I1.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> G_t_I1 = G_T_I1.block(0, 3, 3, 1);

    /// IMU pose at time stamp k
    Pose *imuPose_k = current_state->_clones_IMU.at(timestamp);
    Eigen::Matrix<double, 3, 3> Ik_R_G = imuPose_k->Rot(); // Ik_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ik = imuPose_k->pos(); // p_Ik in G

    /// L0_R_Lk, L0_t_Lk
    Eigen::Matrix3d L1_R_Lk = L1_T_Lk.block(0, 0, 3, 3);
    Eigen::Vector3d L1_t_Lk = L1_T_Lk.block(0, 3, 3, 1);

    /// IMU to LIDAR extrinsic calibration
    Pose *calibration = current_state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_t_L = calibration->pos();

    std::vector<Type*> x_order;
    int total_hx = 0;

    /// Add the clone i: IMU pose corresponding prev update time
    x_order.push_back(imuPose_k);
    total_hx += imuPose_k->size();

    if(current_state->_options.do_calib_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration);
        total_hx += calibration->size();
    }

    /// Assign Jacobian blocks

    /// H2 block, corresponding to res2 (translation residual)
    Eigen::MatrixXd H2_xk_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H2_xk_trans = Eigen::MatrixXd::Zero(3, 3);
    H2_xk_rot = -I_R_L.transpose()*G_R_I1.transpose()*Ik_R_G.transpose()*skew_x(I_t_L);
    H2_xk_trans = I_R_L.transpose()*G_R_I1.transpose();

    Eigen::MatrixXd H2_xc_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H2_xc_trans = Eigen::MatrixXd::Zero(3, 3);
    H2_xc_rot = -I_R_L.transpose()*skew_x(G_R_I1.transpose()*(G_p_Ik-G_t_I1) +
            (G_R_I1.transpose()*Ik_R_G.transpose() - Eigen::Matrix3d::Identity())*I_t_L) +
                    I_R_L.transpose()*(G_R_I1.transpose()*Ik_R_G.transpose()- Eigen::Matrix3d::Identity())*skew_x(I_t_L);
    H2_xc_trans = I_R_L.transpose()*(G_R_I1.transpose()*Ik_R_G.transpose() - Eigen::Matrix3d::Identity());

    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(3, total_hx);

    H_x.block(0, 0, 3, 3) = H2_xk_rot;
    H_x.block(0, 3, 3, 3) = H2_xk_trans;

    if(current_state->_options.do_calib_extrinsic) {
        H_x.block(0, 6, 3, 3) = H2_xc_rot;
        H_x.block(0, 9, 3, 3) = H2_xc_trans;
    }

    /// Predicted measurement
    Eigen::Matrix<double, 3, 1> L1_t_Lk_hat = I_R_L.transpose()*( G_R_I1.transpose() * Ik_R_G.transpose() * I_t_L +
                                                                  G_R_I1.transpose() * (G_p_Ik - G_t_I1) - I_t_L); // Predicted translation

    /// Residual calculation
    Eigen::Matrix<double, 3, 1> res_trans =  L1_t_Lk - L1_t_Lk_hat;

    Eigen::MatrixXd R = std::pow(_options.noise_translation, 2)*Eigen::MatrixXd::Identity(3, 3);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res_trans.dot(S.llt().solve(res_trans));

    boost::math::chi_squared chi_squared_dist(3); // I know res_trans.rows() is 3
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);

    /// TODO: I actually need to check if the dimension should be 3 or not, what does the dimension really mean physically?
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update] unsuccessful updateScan2GlobalMap\n" RESET);
            did_update = false;
            return;
        }
    }

    /// Update
    StateHelper::EKFUpdate(current_state, x_order, H_x, res_trans, R);
    printf(BOLDGREEN "[Update] successful updateScan2GlobalMap\n" RESET);
    did_update = true;
}