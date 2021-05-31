//
// Created by usl on 12/9/20.
//
#ifndef LINCALIB_PARSE_ROS_H
#define LINCALIB_PARSE_ROS_H

#include <ros/ros.h>
#include <core/lincalibManagerOptions.h>

namespace lin_estimator {
    /// This function will load parameters from the ros node handler / parameter server
    lincalibManagerOptions parse_ros_nodehandler(ros::NodeHandle &nh) {
        /// Our lincalib manager options with defaults
        lincalibManagerOptions params;

        /// Estimator
        // Main EKF parameters
        ROS_INFO_STREAM("Reading General Estimator Parameters");
        nh.param<bool>("use_fej", params.state_options.do_fej, params.state_options.do_fej);
        nh.param<bool>("use_imuavg", params.state_options.imu_avg, params.state_options.imu_avg);
        nh.param<bool>("use_rk4int", params.state_options.use_rk4_integration, params.state_options.use_rk4_integration);
        nh.param<bool>("calib_lidar_timeoffset", params.state_options.do_calib_lidar_timeoffset, params.state_options.do_calib_lidar_timeoffset);
        nh.param<int>("max_clones", params.state_options.max_clone_size, params.state_options.max_clone_size);
        nh.param<bool>("calib_extrinsics", params.state_options.do_calib_extrinsic, params.state_options.do_calib_extrinsic);
        nh.param<double>("state_init_x_noise", params.state_options.trans_x_noise, params.state_options.trans_x_noise);
        nh.param<double>("state_init_y_noise", params.state_options.trans_y_noise, params.state_options.trans_y_noise);
        nh.param<double>("state_init_z_noise", params.state_options.trans_z_noise, params.state_options.trans_z_noise);
        nh.param<double>("state_init_rx_noise", params.state_options.rot_x_noise, params.state_options.rot_x_noise);
        nh.param<double>("state_init_ry_noise", params.state_options.rot_y_noise, params.state_options.rot_y_noise);
        nh.param<double>("state_init_rz_noise", params.state_options.rot_z_noise, params.state_options.rot_z_noise);
        nh.param<double>("state_init_timeoffset_noise", params.state_options.time_offset_noise, params.state_options.time_offset_noise);

        /// Filter initialization
        ROS_INFO_STREAM("Reading Filter Initialization Parameters");
        nh.param<double>("init_window_time", params.init_window_time,
                         params.init_window_time);
        nh.param<double>("init_imu_thresh", params.init_imu_thresh,
                         params.init_imu_thresh);

        /// Noise
        // Our noise values for inertial sensor
        ROS_INFO_STREAM("Reading IMU Noise Parameters");
        nh.param<double>("gyroscope_noise_density", params.imu_noises.sigma_w,
                         params.imu_noises.sigma_w);
        nh.param<double>("accelerometer_noise_density", params.imu_noises.sigma_a,
                         params.imu_noises.sigma_a);
        nh.param<double>("gyroscope_random_walk", params.imu_noises.sigma_wb,
                         params.imu_noises.sigma_wb);
        nh.param<double>("accelerometer_random_walk", params.imu_noises.sigma_ab,
                         params.imu_noises.sigma_ab);

        // Read update parameters
        ROS_INFO_STREAM("Reading Updater Chi2 Multiplier");
        nh.param<int>("updater_chi2_multiplier", params.updaterOptions.chi2_multiplier, params.updaterOptions.chi2_multiplier);
        nh.param<bool>("updater_do_chi2_check", params.updaterOptions.do_chi2_check, params.updaterOptions.do_chi2_check);

        ROS_INFO_STREAM("Reading Rotation and Noise Update");
        nh.param<double>("updater_rotation_noise", params.updaterOptions.noise_rotation, params.updaterOptions.noise_rotation);
        nh.param<double>("updater_translation_noise", params.updaterOptions.noise_translation, params.updaterOptions.noise_translation);

        /// Global gravity
        ROS_INFO_STREAM("Reading Gravity");
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        nh.param<std::vector<double>>("gravity", gravity, gravity);
        assert(gravity.size() == 3);
        params.gravity << gravity.at(0), gravity.at(1), gravity.at(2);

        /// State
        // Timeoffset from lidar to IMU
        ROS_INFO_STREAM("Reading initial Timeoffset");
        nh.param<double>("calib_lidarimu_dt", params.calib_lidarimu_dt, params.calib_lidarimu_dt);

        /// Our camera extrinsics transform
        Eigen::Matrix4d I_T_L;
        std::vector<double> matrix_I_T_L;
        std::vector<double> matrix_I_T_L_default = {1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1};
        // Read in from ROS, and save into our eigen mat
//        ROS_INFO_STREAM("Reading initial I_T_L");
//        nh.param<std::vector<double>>("I_T_L", matrix_I_T_L, matrix_I_T_L_default);
//        I_T_L << matrix_I_T_L.at(0), matrix_I_T_L.at(1), matrix_I_T_L.at(2), matrix_I_T_L.at(3),
//                 matrix_I_T_L.at(4), matrix_I_T_L.at(5), matrix_I_T_L.at(6), matrix_I_T_L.at(7),
//                 matrix_I_T_L.at(8), matrix_I_T_L.at(9), matrix_I_T_L.at(10), matrix_I_T_L.at(11),
//                 matrix_I_T_L.at(12), matrix_I_T_L.at(13), matrix_I_T_L.at(14), matrix_I_T_L.at(15);

//        // Load these into our state
//        Eigen::Matrix<double,7,1> lidar_imu_extrinsics_ITL;
//        lidar_imu_extrinsics_ITL.block(0,0,4,1) =
//                rot_2_quat(I_T_L.block(0,0,3,3));
//        lidar_imu_extrinsics_ITL.block(4,0,3,1) =
//                I_T_L.block(0,3,3,1);
//        params.lidar_imu_extrinsics = lidar_imu_extrinsics_ITL;


        /// NDT Resolution
        ROS_INFO_STREAM("Reading NDT Resolution");
        nh.param("ndt_resolution", params.ndtResolution, params.ndtResolution);

        /// Undistortion Flag
        ROS_INFO_STREAM("Reading Undistortion Flag");
        nh.param("do_undistortion", params.do_undistortion, params.do_undistortion);

        ROS_INFO_STREAM("Reading lin output file names");
        nh.param("lin_trajectory_filename", params.lin_trajectory_filename, params.lin_trajectory_filename);

        nh.param("lin_bias_filename", params.lin_bias_filename, params.lin_bias_filename);
        nh.param("lin_velocity_filename", params.lin_velocity_filename, params.lin_velocity_filename);
        nh.param("lin_calib_extrinsic_filename", params.lin_calib_extrinsic_filename, params.lin_calib_extrinsic_filename);
        nh.param("lin_calib_dt_filename", params.lin_calib_dt_filename, params.lin_calib_dt_filename);

        ROS_INFO_STREAM("Reading lo output trajectory file name");
        nh.param("lo_trajectory_filename", params.lo_trajectory_filename, params.lo_trajectory_filename);

        if(params.do_undistortion) {
            params.lin_trajectory_filename = params.lin_trajectory_filename + "_undistortedcloud.csv";
            params.lin_bias_filename = params.lin_bias_filename + "_undistortedcloud.csv";
            params.lin_velocity_filename = params.lin_velocity_filename + "_undistortedcloud.csv";
            params.lin_calib_extrinsic_filename = params.lin_calib_extrinsic_filename + "_undistortedcloud.csv";
            params.lin_calib_dt_filename = params.lin_calib_dt_filename + "_undistortedcloud.csv";
            params.lo_trajectory_filename = params.lo_trajectory_filename + "_undistortedcloud.csv";
        } else {
            params.lin_trajectory_filename = params.lin_trajectory_filename + "_rawcloud.csv";
            params.lin_bias_filename = params.lin_bias_filename + "_rawcloud.csv";
            params.lin_velocity_filename = params.lin_velocity_filename + "_rawcloud.csv";
            params.lin_calib_extrinsic_filename = params.lin_calib_extrinsic_filename + "_rawcloud.csv";
            params.lin_calib_dt_filename = params.lin_calib_dt_filename + "_rawcloud.csv";
            params.lo_trajectory_filename = params.lo_trajectory_filename + "_rawcloud.csv";
        }

        /// File to read the initial calibration result from
        ROS_INFO_STREAM("Reading the filename to read the initial calibration result to");
        nh.param("init_calibration_result_filename", params.init_calibration_result_filename, params.init_calibration_result_filename);

        std::ifstream initial_calib(params.init_calibration_result_filename);
        std::string word;
        int i = 0; int j = 0;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        while (initial_calib >> word){
            T(i, j) = atof(word.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }

        // Read in from ROS, and save into our eigen mat
        ROS_INFO_STREAM("Reading initial I_T_L");
//        nh.param<std::vector<double>>("I_T_L", matrix_I_T_L, matrix_I_T_L_default);
//        I_T_L << matrix_I_T_L.at(0), matrix_I_T_L.at(1), matrix_I_T_L.at(2), matrix_I_T_L.at(3),
//                matrix_I_T_L.at(4), matrix_I_T_L.at(5), matrix_I_T_L.at(6), matrix_I_T_L.at(7),
//                matrix_I_T_L.at(8), matrix_I_T_L.at(9), matrix_I_T_L.at(10), matrix_I_T_L.at(11),
//                matrix_I_T_L.at(12), matrix_I_T_L.at(13), matrix_I_T_L.at(14), matrix_I_T_L.at(15);
        I_T_L = T;
        // Load these into our state
        Eigen::Matrix<double,7,1> lidar_imu_extrinsics_ITL;
        lidar_imu_extrinsics_ITL.block(0,0,4,1) =
                rot_2_quat(I_T_L.block(0,0,3,3));
        lidar_imu_extrinsics_ITL.block(4,0,3,1) =
                I_T_L.block(0,3,3,1);
        params.lidar_imu_extrinsics = lidar_imu_extrinsics_ITL;

        /// File to write the final calibration result to
        ROS_INFO_STREAM("Reading the filename to write the final calibration result to");
        nh.param("calibration_result_filename", params.calibration_result_filename, params.calibration_result_filename);

        /// Scan output folder names
        ROS_INFO_STREAM("Reading the folder name to write the raw and deskewed scan");
        nh.param("raw_scan_folder_name", params.raw_scan_folder_name, params.raw_scan_folder_name);
        nh.param("deskewed_scan_folder_name", params.deskewed_scan_folder_name, params.deskewed_scan_folder_name);

        /// Limit map size params
        nh.param("limit_map_size", params.limit_map_size, params.limit_map_size);
        nh.param("no_of_scans_for_map", params.no_of_scans_for_map, params.no_of_scans_for_map );

        nh.param("downsample_for_mapping", params.downSampleForMapping, params.downSampleForMapping);
        nh.param("gen_map_data", params.gen_map_data, params.gen_map_data);

        /// Batch Optimization related
        nh.param("gen_data_for_optimization", params.gen_data_for_optimization, params.gen_data_for_optimization);
        return params;
    }
}
#endif //LINCALIB_PARSE_ROS_H

