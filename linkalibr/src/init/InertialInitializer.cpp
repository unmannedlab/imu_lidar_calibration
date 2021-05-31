//
// Created by usl on 11/28/20.
//

#include "InertialInitializer.h"

using namespace lin_core;

void InertialInitializer::feed_imu(double timestamp, Eigen::Matrix<double, 3, 1> wm, Eigen::Matrix<double, 3, 1> am) {
    /// Create our IMU data object
    IMUDATA data;
    data.timestamp = timestamp;
    data.wm = wm;
    data.am = am;

    /// Append it our vector
    imu_data.emplace_back(data);

    /// Delete all measurements older than three of our initialization windows
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end() && it0->timestamp < timestamp - 3*_window_length) {
        it0 = imu_data.erase(it0);
    }
}

bool InertialInitializer::initialize_with_imu(double &time0, /// Timestamp that the returned state is at
                                              Eigen::Matrix<double, 4, 1> &q_GtoI0, /// q_GtoI0 Orientation at the initialization
                                              Eigen::Matrix<double, 3, 1> &b_w0, /// b_w0 gyro bias at initialization
                                              Eigen::Matrix<double, 3, 1> &v_I0inG, /// v_I0inG Velocity at initialization
                                              Eigen::Matrix<double, 3, 1> &b_a0, /// b_a0 Acceleration bias at initialization
                                              Eigen::Matrix<double, 3, 1> &p_I0inG, /// p_I0inG Position at initialization
                                              bool wait_for_jerk) /// wait_for_jerk if true we will wait for a "jerk"
{
    /// Return if we do not have any measurements
    if(imu_data.empty()) {
        printf(YELLOW "[InertialInitializer::initialize_with_imu] Cannot attempt to initialize as we have no imu data \n" RESET);
        return false;
    }

    /// Newest IMU timestamp
    double newesttime = imu_data.at(imu_data.size()-1).timestamp;

    /// First, lets collect two windows of IMU readings
    /// One is from the newest measurement to 1 window_length back in time
    /// Second is from 1 window_length back in time to 2 window_lengths back in time
    std::vector<IMUDATA> window_newest, window_secondnew;
    for(IMUDATA data : imu_data) {
        if(data.timestamp > newesttime - 1*_window_length && data.timestamp <= newesttime) {
            window_newest.emplace_back(data);
        }
        if(data.timestamp > newesttime - 2*_window_length && data.timestamp <= newesttime - 1*_window_length) {
            window_secondnew.emplace_back(data);
        }
    }

    /// Return if either of the windows is empty
    if(window_secondnew.empty() || window_newest.empty()) {
        printf(YELLOW "[InertialInitializer::initialize_with_imu] One of the windows is empty, not enough readings \n" RESET);
        return false;
    }

    /// Calculate the sample variance for the newest window
    Eigen::Matrix<double, 3, 1> a_avg = Eigen::Matrix<double, 3, 1>::Zero();
    for(IMUDATA data : window_newest) {
        a_avg += data.am;
    }
    a_avg /= (int)window_newest.size();
    double a_var = 0;
    for(IMUDATA data : window_newest){
        a_var += (data.am - a_avg).dot(data.am - a_avg);
    }
    a_var /= ((int)window_newest.size()-1);
    double a_stdev = std::sqrt(a_var);

    /// If the above standard dev is below a threshold and we are waiting for a "jerk" then we want to wait till it is exceeded
    if(a_stdev < _imu_excite_threshold && wait_for_jerk) {
        printf(YELLOW "[InertialInitializer::initialize_with_imu] Not enough IMU excitation, below threshold %0.4f < %0.4f\n" RESET, a_var, _imu_excite_threshold);
        return false;
    }

    /// Sum of the linear and angular readings from the window with older data (the secondnew window)
    Eigen::Matrix<double, 3, 1> linsum = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> angsum = Eigen::Matrix<double, 3, 1>::Zero();
    for(IMUDATA data : window_secondnew) {
        linsum += data.am;
        angsum += data.wm;
    }
    Eigen::Matrix<double, 3, 1> linavg = linsum/window_secondnew.size();
    Eigen::Matrix<double, 3, 1> angavg = angsum/window_secondnew.size();
    /// Calculate variance and stdev of the second window
    double a_var2 = 0;
    for(IMUDATA data : window_secondnew) {
        a_var2 += (data.am - linavg).dot((data.am - linavg));
    }
    a_var2 /= ((int)window_secondnew.size()-1);
    double a_stdev2 = std::sqrt(a_var2);

    /// If the second window is above the excitation threshold and we are not waiting for a jerk to happen then
    /// We must be stationary, so we return from here and wait until we are stationary
    if((a_stdev > _imu_excite_threshold || a_stdev2 > _imu_excite_threshold) && !wait_for_jerk) {
        printf(YELLOW "[InertialInitializer::initialize_with_imu] too much IMU excitation when we are not waiting for jerk, above threshold %.4f,%.4f > %.4f\n" RESET,a_var,a_var2,_imu_excite_threshold);
        return false;
    }

    /// Get Z axis
    Eigen::Matrix<double, 3, 1> z_axis = linavg/linavg.norm();
    /// Create an X axis
    Eigen::Matrix<double, 3, 1> e_1;
    e_1 << 1, 0, 0;
    Eigen::Matrix<double, 3, 1> x_axis = Eigen::Matrix<double, 3, 1>::Zero();
    x_axis = e_1 - z_axis*z_axis.transpose()*e_1;
    x_axis = x_axis/x_axis.norm();
    /// Create an Y axis
    Eigen::Matrix<double, 3, 1> y_axis = Eigen::Matrix<double, 3, 1>::Zero();
    y_axis = skew_x(z_axis)*x_axis;

    /// From these get the Rotation matrix
    Eigen::Matrix<double, 3, 3> Ro;
    Ro.block(0, 0, 3, 1) = x_axis;
    Ro.block(0, 1, 3, 1) = y_axis;
    Ro.block(0, 2, 3, 1) = z_axis;

    /// Create our state variables
    Eigen::Matrix<double, 4, 1> q_GtoI = rot_2_quat(Ro);

    /// Set the biases equal to the sample mean calculated
    Eigen::Matrix<double, 3, 1> bg = angavg;
    Eigen::Matrix<double, 3, 1> ba = linavg - Ro*_gravity;

    /// Set our state variables
    time0 = window_secondnew.at(window_secondnew.size()-1).timestamp;
    q_GtoI0 = q_GtoI;
    b_w0 = bg;
    v_I0inG = Eigen::Matrix<double, 3, 1>::Zero();
    b_a0 = ba;
    p_I0inG = Eigen::Matrix<double, 3, 1>::Zero();

    printf(GREEN "[InertialInitializer::initialize_with_imu] Initialization done!!!!\n");
    return true;
}