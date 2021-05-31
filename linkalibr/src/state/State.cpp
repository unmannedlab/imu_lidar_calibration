//
// Created by usl on 11/6/20.
//

#include "State.h"

using namespace lin_core;
using namespace lin_estimator;

State::State(StateOptions &options) {
    // Save our options
    _options = options;

    // Append the imu to the state and covariance
    int current_id = 0;
    _imu = new IMU();
    _imu->set_local_id(current_id);
    _variables.push_back(_imu);
    current_id += _imu->size();

    // Camera to IMU time offset
    _calib_dt_LIDARtoIMU = new Vec(1);
    if (_options.do_calib_lidar_timeoffset) {
        _calib_dt_LIDARtoIMU->set_local_id(current_id);
        _variables.push_back(_calib_dt_LIDARtoIMU);
        current_id += _calib_dt_LIDARtoIMU->size();
    }

    // Allocate extrinsic transform

    _calib_LIDARtoIMU = new Pose();
    _calib_LIDARtoIMU->set_local_id(current_id);
    _variables.push_back(_calib_LIDARtoIMU);
    current_id += _calib_LIDARtoIMU->size();


    // Finally initialize our covariance to small value
    _Cov = 1e-3*Eigen::MatrixXd::Identity(current_id, current_id);

    // Finally, set some of our priors for our calibration parameters
    if (_options.do_calib_lidar_timeoffset) {
        _Cov(_calib_dt_LIDARtoIMU->id(),_calib_dt_LIDARtoIMU->id()) = std::pow(_options.time_offset_noise,2);
    }

    if(_options.do_calib_extrinsic) {
        double rot_x_cov = std::pow(_options.rot_x_noise, 2);
        double rot_y_cov = std::pow(_options.rot_y_noise, 2);
        double rot_z_cov = std::pow(_options.rot_z_noise, 2);
        Eigen::MatrixXd Rot_cov = Eigen::MatrixXd::Identity(3, 3);
        Rot_cov(0, 0) = rot_x_cov;
        Rot_cov(1, 1) = rot_y_cov;
        Rot_cov(2, 2) = rot_z_cov;

        double trans_x_cov = std::pow(_options.trans_x_noise, 2);
        double trans_y_cov = std::pow(_options.trans_y_noise, 2);
        double trans_z_cov = std::pow(_options.trans_z_noise, 2);
        Eigen::MatrixXd Trans_cov = Eigen::MatrixXd::Identity(3, 3);
        Trans_cov(0, 0) = trans_x_cov;
        Trans_cov(1, 1) = trans_y_cov;
        Trans_cov(2, 2) = trans_z_cov;

        std::cout << "Rot_cov \n" << std::endl;
        std::cout << Rot_cov << std::endl;

        std::cout << "Trans_cov \n" << std::endl;
        std::cout << Trans_cov << std::endl;

        _Cov.block(_calib_LIDARtoIMU->id(),_calib_LIDARtoIMU->id(),3,3) = Rot_cov;
        _Cov.block(_calib_LIDARtoIMU->id()+3,_calib_LIDARtoIMU->id()+3,3,3) = Trans_cov;
    }
}