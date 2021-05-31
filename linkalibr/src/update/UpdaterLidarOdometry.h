//
// Created by usl on 11/28/20.
//

#ifndef LIN_ESTIMATOR_UPDATERHEC_H
#define LIN_ESTIMATOR_UPDATERHEC_H

#include <Eigen/Eigen>
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/color.h"

#include "UpdaterOptions.h"

#include "relpose/relativePose.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace lin_estimator {
    /// Will compute the system for the lidar odometry measurement and update the filter
    class UpdaterLidarOdometry {
    public:
        UpdaterLidarOdometry(UpdaterOptions &options) : _options(options) {
            for(int i = 1; i < 500; i++) {
                boost::math::chi_squared chi_squared_dist(i);
                chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
            }
        }

        /// Given lidar odometry, this will use them to update the state
        /// state: The current state of the system
        /// lodom: relative lidar odometry result that can be used for update
        void updateScan2Scan(State *current_state, relativePose lodom, bool &did_update);

        /// Given lidar odometry, this will use them to update the state
        /// state: The current state of the system
        /// L0_T_Lk: global lidar odometry result that can be used for update
        void updateScan2GlobalMap(State *current_state, Eigen::Matrix4d L0_T_Lk, Eigen::Matrix4d G_T_I1, double timestamp, bool &did_update);

//        /// Find the clone index closest to the queried timestamp
//        static double findClosestCloneTimeStamp(State *state, double timestamp);

     protected:

        /// Options used during update
        UpdaterOptions _options;

        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_squared_table;

    };
};
#endif //LIN_ESTIMATOR_UPDATERHEC_H
