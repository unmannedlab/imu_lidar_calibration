//
// Created by usl on 1/3/21.
//

#ifndef LINKALIBR_PLANEDETECTOR_H
#define LINKALIBR_PLANEDETECTOR_H

#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include "utils/math_utils.h"
#include "utils/eigen_utils.h"
#include "utils/quat_ops.h"
#include "utils/pcl_utils.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/ModelCoefficients.h>

#include <track/PlaneDetectorParams.h>

namespace lin_core {
    class PlaneDetector {
    public:
        PlaneDetector(PlaneDetectorParams _params) {
            _first_frame = true;
            std::cout << "--PlaneTargetDetector Initialized!--" << std::endl;
            _params.print();
        }

        void feedData();

    private:
        bool _first_frame;
    };
}
#endif //LINKALIBR_PLANEDETECTOR_H
