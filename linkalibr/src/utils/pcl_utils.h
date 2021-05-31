//
// Created by usl on 12/27/20.
//

#ifndef LINKALIBR_PCL_UTILS_H
#define LINKALIBR_PCL_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Eigen>

namespace lin_core {

    typedef pcl::PointXYZI VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

    typedef pcl::PointXYZRGB colorPointT;
    typedef pcl::PointCloud<colorPointT> colorPointCloudT;

    struct PointXYZIR8Y {
        PCL_ADD_POINT4D
        ; // quad-word XYZ
        float intensity; ///< laser intensity reading
        uint8_t ring; ///< laser ring number
        uint32_t t;
        float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    } EIGEN_ALIGN16;

    inline void downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                                float in_leaf_size) {
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(in_cloud);
        sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
        sor.filter(*out_cloud);
    }
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        lin_core::PointXYZIR8Y,
             (float, x, x)
             (float, y, y)
             (float, z, z)
             (float, intensity, intensity)
             (uint8_t, ring, ring)
             (uint32_t, t, t))

typedef lin_core::PointXYZIR8Y TPoint;
typedef pcl::PointCloud<TPoint> TPointCloud;

namespace lin_core {
    inline void TPointCloud2VPointCloud(TPointCloud::Ptr input_pc,
                                        VPointCloud::Ptr output_pc) {
        output_pc->header = input_pc->header;
        output_pc->height = input_pc->height;
        output_pc->width = input_pc->width;
        output_pc->is_dense = input_pc->is_dense;
        output_pc->resize(output_pc->width * output_pc->height);
        for(int h = 0; h < input_pc->height; h++) {
            for(int w = 0; w < input_pc->width; w++) {
                if(pcl_isnan(input_pc->at(w,h).x)||pcl_isnan(input_pc->at(w,h).y)||pcl_isnan(input_pc->at(w,h).z))
                    continue;
                lin_core::VPoint point;
                point.x = input_pc->at(w,h).x;
                point.y = input_pc->at(w,h).y;
                point.z = input_pc->at(w,h).z;
                point.intensity = input_pc->at(w,h).intensity;
                output_pc->at(w,h) = point;
            }
        }
    }
}

namespace lin_core {
    inline void downsampleCloud(pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr in_cloud,
                                pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr out_cloud,
                                float in_leaf_size) {
        pcl::VoxelGrid<lin_core::PointXYZIR8Y> sor;
        sor.setInputCloud(in_cloud);
        sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
        sor.filter(*out_cloud);
    }
}
#endif //LINKALIBR_PCL_UTILS_H
