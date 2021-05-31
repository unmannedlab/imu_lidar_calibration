//
// Created by usl on 11/28/20.
//

#ifndef LIN_CORE_INERTIALINITIALIZER_H
#define LIN_CORE_INERTIALINITIALIZER_H

#include <Eigen/Eigen>

#include "utils/quat_ops.h"
#include "utils/color.h"

namespace lin_core {
  class InertialInitializer {
  public:
      /// Struct for a single imu measurement
      struct IMUDATA {
          /// Timestamp of the reading
          double timestamp;
          /// Gyroscope reading: angular velocity (rad/s)
          Eigen::Matrix<double, 3, 1> wm;
          /// Accelerometer reading: linear acceleration (m/s^2)
          Eigen::Matrix<double, 3, 1> am;
      };

      /// Default constructor
      InertialInitializer(Eigen::Matrix<double, 3, 1> gravity, double window_length, double imu_excite_threshold):
      _gravity(gravity), _window_length(window_length), _imu_excite_threshold(imu_excite_threshold) {}

      /// Stores incoming imu readings
      void feed_imu(double time_stamp, Eigen::Matrix<double, 3, 1> wm, Eigen::Matrix<double, 3, 1> am);

      /// Try to initialize the system using just the imu
      /// Returns true if we have successfully initialized our system
      bool initialize_with_imu(double &time0, /// Timestamp that the returned state is at
                               Eigen::Matrix<double, 4, 1> &q_GtoI0, /// q_GtoI0 Orientation at the initialization
                               Eigen::Matrix<double, 3, 1> &b_w0, /// b_w0 gyro bias at initialization
                               Eigen::Matrix<double, 3, 1> &v_I0inG, /// v_I0inG Velocity at initialization
                               Eigen::Matrix<double, 3, 1> &b_a0, /// b_a0 Acceleration bias at initialization
                               Eigen::Matrix<double, 3, 1> &p_I0inG, /// p_I0inG Position at initialization
                               bool wait_for_jerk = true); /// wait_for_jerk if true we will wait for a "jerk"

  protected:
      /// Gravity vector
      Eigen::Matrix<double, 3, 1> _gravity;
      /// Amount of time we will initialize over (seconds)
      double _window_length;
      /// Variance threshold on our acceleration to be classified as moving
      double _imu_excite_threshold;
      /// Our history of IMU messages (time, angular, linear)
      std::vector<IMUDATA> imu_data;
  };
};
#endif //LIN_CORE_INERTIALINITIALIZER_H
