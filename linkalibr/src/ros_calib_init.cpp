//
// Created by usl on 3/5/21.
//


#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "utils/quat_ops.h"
#include "utils/pcl_utils.h"

#include  <iostream>
#include "track/lidarOdometry.h"

#include <imuPacket/imuPacket.h>

int main(int argc, char** argv) {
    /// Launch ros node
    ros::init(argc, argv, "ros_pair_lodom_imu");
    ros::NodeHandle nh("~");

    ros::Publisher imu_packet_publisher = nh.advertise<imuPacket::imuPacket>("/imu_packet", 1);
    ros::Publisher lodom_publisher = nh.advertise<geometry_msgs::PoseStamped>("/lidar_odometry", 1);

    /// Our topics (IMU and LIDAR)
    std::string topic_imu;
    std::string topic_lidar;
    double ndt_resolution;
    std::string lo_trajectory_filename;
    std::string imu_data_filename;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");
    nh.param<double>("ndt_resolution", ndt_resolution, 0.5);

    ROS_INFO_STREAM("topic_imu: " << topic_imu);
    ROS_INFO_STREAM("topic_lidar: " << topic_lidar);
    ROS_INFO_STREAM("ndt_resolution: " << ndt_resolution);

    /// Get our start location and how much of the bag we want to play
    /// Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO_STREAM("bag start: " << bag_start);
    ROS_INFO_STREAM("bag duration: " << bag_durr);

    /// Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/usl/datasets/ouster_vectornav.bag");
    ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());

    /// Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    /// We should load the bag as a view
    /// Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    /// Start a few seconds in from the full view time
    /// If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish =
            (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO_STREAM("Time start = " << time_init.toSec());
    ROS_INFO_STREAM("Time end = " << time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);
    /// Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    /// Lidar Odometry object (Tracker)
    lin_core::LidarOdometry::Ptr LOdom;
    LOdom = std::make_shared<lin_core::LidarOdometry>(ndt_resolution, "", true);
    imuPacket::imuPacket imupacket;
    for (const rosbag::MessageInstance& m : view) {
        /// If ROS wants us to stop, break out
        if(!ros::ok())
            break;
        /// Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
        if (s_imu != nullptr && m.getTopic() == topic_imu) {
            imupacket.stamps.push_back(s_imu->header.stamp);
            geometry_msgs::Vector3 accelReading;
            accelReading.x = s_imu->linear_acceleration.x;
            accelReading.y = s_imu->linear_acceleration.y;
            accelReading.z = s_imu->linear_acceleration.z;
            imupacket.accelreadings.push_back(accelReading);
            geometry_msgs::Vector3 gyroReading;
            gyroReading.x = s_imu->angular_velocity.x;
            gyroReading.y = s_imu->angular_velocity.y;
            gyroReading.z = s_imu->angular_velocity.z;
            imupacket.gyroreadings.push_back(gyroReading);
        }

        /// Handle Lidar measurement
        sensor_msgs::PointCloud2::ConstPtr s_lidar = m.instantiate<sensor_msgs::PointCloud2>();
        if (s_lidar != nullptr && m.getTopic() == topic_lidar) {
            lin_core::VPointCloud::Ptr cloud_pcl(new lin_core::VPointCloud);
            pcl::fromROSMsg(*s_lidar, *cloud_pcl);
            LOdom->feedScan((*s_lidar).header.stamp.toSec(), cloud_pcl);
            LOdom->append_and_update(true);
            Eigen::Matrix3d deltaR_L = LOdom->get_latest_relativePose().odometry_ij.block(0, 0, 3, 3);
            Eigen::Quaterniond delta_qL(deltaR_L);
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = s_lidar->header.stamp;
            pose.pose.orientation.x = delta_qL.x();
            pose.pose.orientation.y = delta_qL.y();
            pose.pose.orientation.z = delta_qL.z();
            pose.pose.orientation.w = delta_qL.w();
            lodom_publisher.publish(pose);
            imupacket.header.stamp = s_lidar->header.stamp;
            imu_packet_publisher.publish(imupacket);
            imupacket.stamps.clear();
            imupacket.accelreadings.clear();
            imupacket.gyroreadings.clear();
        }
    }
    return EXIT_SUCCESS;
}