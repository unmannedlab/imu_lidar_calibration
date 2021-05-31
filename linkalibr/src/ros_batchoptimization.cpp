#include "ros/ros.h"
#include <batch_optimization/BatchOptimizer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_batchoptimization");
    ros::NodeHandle nh("~");
    lin_estimator::OptimizerParams params;

    nh.param<std::string>("imu_measurements_csv_filename", params.imu_measurements_csv_filename, "");
    nh.param<std::string>("imu_trajectory_csv_filename", params.imu_trajectory_csv_filename, "");
    nh.param<std::string>("imu_trajectory_out_csv_filename", params.imu_trajectory_out_csv_filename, "");
    nh.param<std::string>("plane_params_csv_filename", params.plane_params_csv_filename,"");
    nh.param<std::string>("planar_points_csv_filename", params.planar_points_csv_filename,"");
    nh.param<std::string>("planar_points_deskewed_csv_filename", params.planar_points_deskewed_csv_filename,"");
    nh.param<std::string>("initial_calib_filename", params.initial_calib_filename,"");
    nh.param<std::string>("init_calib_csv_filename", params.init_calib_csv_filename,"");
    nh.param<std::string>("final_calib_csv_filename", params.final_calib_csv_filename,"");
    nh.param<std::string>("imu_velocity_out_csv_filename", params.imu_velocity_out_csv_filename,"");

    nh.param<double>("accelerometer_noise_density", params.accelerometer_noise_density, 0.0001);
    nh.param<double>("accelerometer_random_walk", params.accelerometer_random_walk, 0.0001);
    nh.param<double>("gyroscope_noise_density", params.gyroscope_noise_density, 0.0001);
    nh.param<double>("gyroscope_random_walk", params.gyroscope_random_walk, 0.0001);

    lin_estimator::BatchOptimizer batch_optimizer(params);

    ros::spin();
    return 0;
}


