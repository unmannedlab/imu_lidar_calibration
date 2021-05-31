//
// Created by usl on 11/29/20.
//

#ifndef LIN_CORE_RELATIVEPOSE_H
#define LIN_CORE_RELATIVEPOSE_H

#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include "types/Pose.h"

using namespace lin_type;

namespace lin_core {
    class relativePose {
    public:
        /// Time stamp of scan i
        double timestamp_i;
        /// Time stamp of scan j
        double timestamp_j;
        /// Odometry pose
        Eigen::Matrix4d odometry_ij;
    };
}
#endif //LIN_CORE_RELATIVEPOSE_H
