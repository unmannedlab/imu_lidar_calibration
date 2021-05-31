//
// Created by usl on 11/6/20.
//

#ifndef LINCALIB_STATE_H
#define LINCALIB_STATE_H

#include <vector>
#include <unordered_map>

#include "types/Type.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "types/Pose.h"
#include "StateOptions.h"


using namespace lin_core;
using namespace lin_type;

namespace lin_estimator {
  class State {
  public:
      State(StateOptions &options_);
      ~State() {}

      /// Returns timestep of the clone that we will marginalize
      double margtimestep() {
          double time = INFINITY;
          for (std::pair<const double, Pose*> &clone_imu : _clones_IMU) {
              if(clone_imu.first < time) {
                  time = clone_imu.first;
              }
          }
          return time;
      }

      /// Calculates the current max size of the covariance
      int max_covariance_size() {
        return (int)_Cov.rows();
      }

      /// Current timestamp (should be the last update time)
      double _timestamp;

      /// Struct constaining filter options
      StateOptions _options;

      /// Pointer to active IMU State (q_GtoI, p_IinG, V_IinG, bg, ba)
      IMU *_imu;

      /// Map between scanning times and clone poses (q_GtoIi, p_IiinG)
      std::map<double, Pose*> _clones_IMU;

      /// Time offset between base IMU to lidar (t_imu = t_lidar + t_offset)
      Vec *_calib_dt_LIDARtoIMU;

      /// Calib pose for IMU Lidar system I_R_L, I_t_L
      Pose* _calib_LIDARtoIMU;

  private:
      /// Define that the state helper is a friend class of this class
      /// So that it can access the following private members
      friend class StateHelper;
      /// Covariance of all active variables
      Eigen::MatrixXd _Cov;
      /// Vector of variables
      std::vector<Type*> _variables;
  };
};
#endif //LINCALIB_STATE_H
