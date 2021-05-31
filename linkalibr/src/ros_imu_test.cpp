#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include  <iostream>
#include <fstream>
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv) {
    /// Launch ros node
    ros::init(argc, argv, "ros_pair_lodom_imu");
    ros::NodeHandle nh("~");

    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>("/imu", 1);

    /// Our topics (IMU)
    std::string topic_imu;
    std::string lo_trajectory_filename;
    std::string imu_data_filename;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("imu_data_filename", imu_data_filename, "/home/usl/catkin_ws/src/linkalibr/data/imu_test_data.csv");

    ROS_INFO_STREAM("topic_imu: " << topic_imu);

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

    std::ofstream imu_file_writer;
    imu_file_writer.open(imu_data_filename);

    bool first_imu = true;
    sensor_msgs::Imu prev_imu;
    double deltaT = 0;
    std::vector<double> all_deltaT;
//    int counter_greater = 0;
//    int counter_lesser = 0;
    for (const rosbag::MessageInstance& m : view) {
        /// If ROS wants us to stop, break out
        if(!ros::ok())
            break;
        /// Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
        if (s_imu != nullptr && m.getTopic() == topic_imu) {
            imu_publisher.publish(*s_imu);
            imu_file_writer << (*s_imu).header.stamp.toNSec() << "," << s_imu->angular_velocity.x << ", "<< s_imu->angular_velocity.y << ", "<< s_imu->angular_velocity.z
                                                              << "," << s_imu->linear_acceleration.x << ", "<< s_imu->linear_acceleration.y << ", "<< s_imu->linear_acceleration.z <<"\n";
            if(first_imu) {
                first_imu = false;
            } else {
                deltaT = (s_imu->header.stamp.toSec() - prev_imu.header.stamp.toSec())*1000;
                std::cout << deltaT << std::endl;
//                if(deltaT > 2.5)
//                    counter_greater++;
//                else
//                    counter_lesser++;
                all_deltaT.push_back(deltaT);
            }
            prev_imu = *s_imu;
        }
    }
    double sum = std::accumulate(all_deltaT.begin(), all_deltaT.end(), 0.0);
    double mean = sum / all_deltaT.size();

    std::vector<double> diff(all_deltaT.size());
    std::transform(all_deltaT.begin(), all_deltaT.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / all_deltaT.size());
    std::cout << "Mean: " << mean << std::endl;
    std::cout << "Std-dev: " << stdev << std::endl;
//    std::cout << "counter_greater: " << counter_greater << std::endl;
//    std::cout << "counter_lesser: " << counter_lesser << std::endl;
    return EXIT_SUCCESS;
}