//
// Created by usl on 12/10/20.
//

#ifndef LINCALIB_LIDARODOMETRY_H
#define LINCALIB_LIDARODOMETRY_H

#include "utils/math_utils.h"
#include "utils/eigen_utils.h"
#include "utils/quat_ops.h"
#include "utils/pcl_utils.h"

#include <pclomp/ndt_omp.h>

#include "types/Pose.h"
#include "relpose/relativePose.h"

#include <fstream>

namespace lin_core {
    class LidarOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<LidarOdometry> Ptr;

        struct Odom {
            double timestamp;
            Eigen::Matrix4d pose; // current scan to first scan
        };

        explicit LidarOdometry(double ndtResolution = 0.5,
                               std::string lo_trajectory_filename="",
                               bool downsample_for_mapping = false);

        static pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndtInit(
                double ndt_resolution);

        void feedScan(double timestamp,
                      VPointCloud::Ptr cur_scan,
                      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity());

        void detectPlanarTargetInFirstFrame(double timestamp);

        void clearOdometryData();

        void setTargetMap(VPointCloud::Ptr map_cloud_in);

        void saveTargetMap(const std::string& path) const {
            ROS_INFO_STREAM("Save NDT target map to " << path
                                                      << "; size: " << map_cloud_->size());
            pcl::io::savePCDFileASCII(path, *map_cloud_);
        }

        const VPointCloud::Ptr getTargetMap() {
            return map_cloud_;
        }

        const VPointCloud::Ptr getScanInTarget() {
            return scan_in_target_global_;
        }

        const pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr& getNDTPtr() const {
            return ndt_omp_;
        }

        const Eigen::aligned_vector<Odom> &get_odom_data() const {
            return odom_;
        }

        const Odom &get_current_odometry() {
            return odom_curr;
        }

        const relativePose get_latest_relativePose() {
            return latestRP;
        }

        const Eigen::aligned_vector<Odom> &get_kf_odom_data() const {
            return kf_odom_;
        }

        const bool &isKeyFrame() const {
            return is_KF_;
        }

        void append_and_update(bool update_map = true);

    private:
        void registration(const VPointCloud::Ptr& cur_scan,
                          const Eigen::Matrix4d& pose_predict,
                          Eigen::Matrix4d& pose_out,
                          VPointCloud::Ptr scan_in_target);
        void updateKeyScan(const VPointCloud cur_scan, const Odom& odom);
        bool checkKeyScan(const Odom& odom);

        // Normalize angle to be between [-180, 180]
        static inline double normalize_angle(double ang_degree) {
            if(ang_degree > 180)
                ang_degree -= 360;
            if(ang_degree < -180)
                ang_degree += 360;
            return ang_degree;
        }
    private:
        VPointCloud::Ptr map_cloud_;
        VPointCloud::Ptr scan_in_target_global_;
        pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndt_omp_;

        std::vector<size_t> key_frame_index_;
        Eigen::aligned_vector<Odom> kf_odom_;
        Eigen::aligned_vector<Odom> odom_;
        Odom odom_curr;
        relativePose latestRP; // latest relative pose
        bool is_KF_;

        std::ofstream trajfile_csv;
        VPointCloud current_scan;

        bool first_scan = true;
        bool downsampleForMapping;
    };
}

#endif //LINCALIB_LIDARODOMETRY_H
