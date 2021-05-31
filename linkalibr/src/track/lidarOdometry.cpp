//
// Created by usl on 12/10/20.
//

#include "lidarOdometry.h"

namespace lin_core {

    LidarOdometry::LidarOdometry(double ndt_resolution, std::string lo_trajectory_filename, bool downsample_for_mapping)
            : map_cloud_(new VPointCloud()), scan_in_target_global_(new VPointCloud()) {
        ndt_omp_ = ndtInit(ndt_resolution);
        latestRP.odometry_ij = Eigen::Matrix4d::Identity();
        trajfile_csv.open(lo_trajectory_filename);
        downsampleForMapping = downsample_for_mapping;
    }

    pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
    LidarOdometry::ndtInit(double ndt_resolution) {
        auto ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
                (new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
        ndt_omp->setResolution(ndt_resolution);
        ndt_omp->setNumThreads(16);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_omp->setTransformationEpsilon(1e-3);
        ndt_omp->setStepSize(0.01);
        ndt_omp->setMaximumIterations(50);
        return ndt_omp;
    }

    void LidarOdometry::feedScan(double timestamp,
                                 VPointCloud::Ptr cur_scan,
                                 Eigen::Matrix4d pose_predict) {
        odom_curr.timestamp = timestamp;
        odom_curr.pose = Eigen::Matrix4d::Identity();
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_target(new pcl::PointCloud<pcl::PointXYZI>());
        current_scan = *cur_scan;
        if(map_cloud_->empty()) {
            scan_in_target = cur_scan;
        } else {
            Eigen::Matrix4d T_LM_predict = odom_.back().pose*pose_predict;
            registration(cur_scan, T_LM_predict, odom_curr.pose, scan_in_target);
        }

        if(first_scan) {
            odom_.push_back(odom_curr);
            updateKeyScan(current_scan, odom_curr);
            first_scan = false;
        } else {
            size_t lastIdx = odom_.size() - 1;
            Odom odom_i = odom_[lastIdx];
            Odom odom_j = odom_curr;
            latestRP.timestamp_i = odom_i.timestamp;
            Eigen::Matrix4d w_T_i = odom_i.pose;
            latestRP.timestamp_j = odom_j.timestamp;
            Eigen::Matrix4d w_T_j = odom_j.pose;
            Eigen::Matrix4d i_T_j = w_T_i.inverse()*w_T_j;
            latestRP.odometry_ij = i_T_j;
        }
    }

    void LidarOdometry::append_and_update(bool update_map) {
        Eigen::Matrix3d R_curr = odom_curr.pose.block(0, 0, 3, 3);
        Eigen::Quaterniond quat_curr(R_curr);
        trajfile_csv << odom_curr.timestamp << "," << quat_curr.x() << "," << quat_curr.y() << "," << quat_curr.z() << ","<< quat_curr.w() << ","
                     << odom_curr.pose(0, 3) << "," << odom_curr.pose(1, 3) << ","<< odom_curr.pose(2, 3) << std::endl;

        odom_.push_back(odom_curr);

        if(update_map) {
            updateKeyScan(current_scan, odom_curr);
        }

    }

    void LidarOdometry::registration(const VPointCloud::Ptr& cur_scan,
                                     const Eigen::Matrix4d& pose_predict,
                                     Eigen::Matrix4d& pose_out,
                                     VPointCloud::Ptr scan_in_target) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr p_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        downsampleCloud(cur_scan, p_filtered_cloud, 0.1);

        ndt_omp_->setInputSource(p_filtered_cloud);
        ndt_omp_->align(*scan_in_target, pose_predict.cast<float>());
        pose_out = ndt_omp_->getFinalTransformation().cast<double>();
    }

    void LidarOdometry::updateKeyScan(const VPointCloud cur_scan,
                                      const Odom& odom) {
        if(checkKeyScan(odom)) {
            VPointCloud ::Ptr filtered_cloud(new VPointCloud());
            VPointCloud::Ptr input_scan(new VPointCloud);
            *input_scan = cur_scan;
            if(downsampleForMapping) {
                downsampleCloud(input_scan, filtered_cloud, 0.1);
            } else {
                *filtered_cloud = *input_scan;
            }
            VPointCloud::Ptr scan_in_target(new VPointCloud ());
            pcl::transformPointCloud(*filtered_cloud, *scan_in_target, odom.pose);
            scan_in_target_global_->clear();
            *scan_in_target_global_ = *scan_in_target;
            *map_cloud_ += *scan_in_target;
            ndt_omp_->setInputTarget(map_cloud_);
            key_frame_index_.push_back(odom_.size());
        }
    }

    bool LidarOdometry::checkKeyScan(const Odom &odom) {
        static Eigen::Vector3d position_last(0,0,0);
        static Eigen::Vector3d ypr_last(0, 0, 0);

        Eigen::Vector3d position_now = odom.pose.block<3, 1>(0, 3);
        double dist = (position_now - position_last).norm();

        const Eigen::Matrix3d rotation(odom.pose.block<3, 3>(0, 0));
        Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
        Eigen::Vector3d delta_angle = ypr - ypr_last;
        for (size_t i = 0; i < 3; i++)
            delta_angle(i) = normalize_angle(delta_angle(i));
        delta_angle = delta_angle.cwiseAbs();
        if (key_frame_index_.size() == 0 ||
            dist > 0.2 ||
            delta_angle(0) > 5.0 ||
            delta_angle(1) > 5.0 ||
            delta_angle(2) > 5.0) {
            position_last = position_now;
            ypr_last = ypr;
            is_KF_ = true;
            kf_odom_.push_back(odom);
//            ROS_WARN_STREAM("[LidarOdometry::checkKeyScan] This is a Key Frame, returning true");
            return true;
        }
//        ROS_INFO_STREAM("[LidarOdometry::checkKeyScan] Not a Key Frame, returning false");
        is_KF_ = false;
        return false;
    }

    void LidarOdometry::setTargetMap(VPointCloud::Ptr map_cloud_in) {
        map_cloud_->clear();
        pcl::copyPointCloud(*map_cloud_in, *map_cloud_);
        ndt_omp_->setInputTarget(map_cloud_);
    }

    void LidarOdometry::clearOdometryData() {
        key_frame_index_.clear();
        odom_.clear();
        kf_odom_.clear();
    }

}