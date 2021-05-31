//
// Created by usl on 12/9/20.
//

#include "lincalibManager.h"

using namespace lin_core;
using namespace lin_type;
using namespace lin_estimator;

lin_estimator::lincalibManager::lincalibManager(lincalibManagerOptions& params_) {
    /// Startup  message
    // Nice startup message
    printf("=======================================\n");
    printf("Lidar Inertial Calibration ON-MANIFOLD EKF IS STARTING\n");
    printf("=======================================\n");

    /// Nice debug
    this->params = params_;
    params.print_estimator();
    params.print_noise();
    params.print_state();
    params.print_trackers();

    /// File to store lin trajectory
    trajfile_csv.open(params.lin_trajectory_filename);
    bias_csv.open(params.lin_bias_filename);
    velocity_csv.open(params.lin_velocity_filename);
    calib_extrinsic_csv.open(params.lin_calib_extrinsic_filename);
    calib_dt_csv.open(params.lin_calib_dt_filename);

    /// Create the state
    state = new State(params.state_options);

    // Time-offset from Lidar to IMU
    Eigen::VectorXd temp_lidarimu_dt;
    temp_lidarimu_dt.resize(1);
    temp_lidarimu_dt(0) = params.calib_lidarimu_dt;
    state->_calib_dt_LIDARtoIMU->set_value(temp_lidarimu_dt);
    state->_calib_dt_LIDARtoIMU->set_fe(temp_lidarimu_dt);

    // Extrinsic Calibration
    state->_calib_LIDARtoIMU->set_value(params.lidar_imu_extrinsics);
    state->_calib_LIDARtoIMU->set_fe(params.lidar_imu_extrinsics);

    /// Propagator
    propagator = new Propagator(params.imu_noises, params.gravity);

    /// Initializer
    initializer = new InertialInitializer(params.gravity, params.init_window_time, params.init_imu_thresh);

    /// Make the Updator
    updaterLidarOdometry = new UpdaterLidarOdometry(params.updaterOptions);
    /// Initialize Lidar Odometry object
    LOdom = std::make_shared<lin_core::LidarOdometry>(params.ndtResolution, params.lo_trajectory_filename, params.downSampleForMapping);
//    LOdom = LidarOdometry(params.ndtResolution, params.lo_trajectory_filename, false);
}

void lin_estimator::lincalibManager::do_undistortion(double timestamp,
                                                     TPointCloud& scan_raw,
                                                     TPointCloud::Ptr& scan_out,
                                                     Eigen::Matrix4d& T_ndt_predict) {
    scan_out->header = scan_raw.header;
    scan_out->is_dense = scan_raw.is_dense;

    /// IMU to LIDAR extrinsic calibration
    Pose *calibration = state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_t_L = calibration->pos();

    Eigen::Matrix<double, 13, 1> imu_state_start;

    double pointStartTimeStamp;
    std::vector<uint32_t > point_timestamps;
    std::map<uint32_t , Eigen::Matrix<double, 13, 1> > stamped_poses;

    ros::Time time_start = ros::Time::now();
    scan_out->header = scan_raw.header;
    scan_out->height = scan_raw.height;
    scan_out->width = scan_raw.width;
    scan_out->is_dense = scan_raw.is_dense;
    scan_out->resize(scan_raw.width*scan_raw.height);
    for(int h = 0; h < scan_raw.height; h++) {
        for(int w = 0; w < scan_raw.width; w++) {
            TPoint scan_point = scan_raw.at(w, h);
            uint32_t point_timestamp = scan_raw.at(w, h).t;
            Eigen::Vector3d skewedPoint = Eigen::Vector3d(scan_point.x, scan_point.y, scan_point.z);
            /// Ignore NaNs
            if(pcl_isnan(scan_point.x) || pcl_isnan(scan_point.y) || pcl_isnan(scan_point.z)) {
                continue;
            }

            if (!point_timestamps.empty()) {
                auto it = find(point_timestamps.begin(), point_timestamps.end(), point_timestamp);
                if (it == point_timestamps.end()) {
                    /// New timestamp
                    point_timestamps.push_back(point_timestamp);
                    double pointCurrTimeStamp = timestamp + point_timestamp/1e9;
                    Eigen::Matrix<double, 13, 1> imu_state_plus;
                    propagator->fast_state_propagate(state, pointCurrTimeStamp, imu_state_plus);
                    stamped_poses.insert(std::make_pair(point_timestamp, imu_state_plus));
                }
            } else {
                /// This is the first point
//                assert(i == 0);
                point_timestamps.push_back(point_timestamp);
                pointStartTimeStamp = timestamp + point_timestamp/1e9;
                propagator->fast_state_propagate(state, pointStartTimeStamp, imu_state_start);
                stamped_poses.insert(std::make_pair(point_timestamp, imu_state_start));
            }
            Eigen::Matrix<double, 13, 1> imu_state_plus = stamped_poses.find(point_timestamp)->second;
            Eigen::Vector3d deskewedPoint = deskewPoint(imu_state_start, imu_state_plus, skewedPoint, I_R_L, I_t_L);
            TPoint deskewed_scan_point;
            deskewed_scan_point.x = deskewedPoint.x();
            deskewed_scan_point.y = deskewedPoint.y();
            deskewed_scan_point.z = deskewedPoint.z();
            deskewed_scan_point.intensity = scan_point.intensity;
            deskewed_scan_point.ring = scan_point.ring;
            deskewed_scan_point.range = scan_point.range;
            scan_out->at(w, h) = deskewed_scan_point;
        }
    }

    auto max_it = max_element(std::begin(point_timestamps), std::end(point_timestamps)); // c++11
    auto min_it = min_element(std::begin(point_timestamps), std::end(point_timestamps)); // c++11
//    ROS_INFO_STREAM("No of time stamps: " << point_timestamps.size());
    Eigen::Matrix<double, 13, 1> imu_state_plus_start;
    propagator->fast_state_propagate(state, timestamp + (double)*min_it/1e9, imu_state_plus_start);
    Eigen::Matrix<double, 4, 1> quat_start = imu_state_plus_start.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 1> pos_start = imu_state_plus_start.block(4, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> start_R_G = lin_core::quat_2_Rot(quat_start);

    Eigen::Matrix<double, 13, 1> imu_state_plus_end;
    propagator->fast_state_propagate(state, timestamp + (double)*max_it/1e9, imu_state_plus_end);
    Eigen::Matrix<double, 4, 1> quat_end = imu_state_plus_end.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 1> pos_end = imu_state_plus_start.block(4, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> end_R_G = lin_core::quat_2_Rot(quat_end);

    Eigen::Matrix<double, 3, 3> start_R_end = start_R_G*end_R_G.transpose();
    Eigen::Matrix<double, 3, 1> start_p_end = start_R_G * (pos_end - pos_start);

    Eigen::Matrix3d Lstart_R_Lend = I_R_L.transpose()*start_R_end*I_R_L;
    Eigen::Vector3d Lstart_t_Lend = I_R_L.transpose()*start_R_end*I_t_L + I_R_L.transpose()*(start_p_end - I_t_L);

    T_ndt_predict = Eigen::Matrix4d::Identity();
    T_ndt_predict.block(0, 0, 3, 3) = Lstart_R_Lend;
    T_ndt_predict.block(0, 3, 3, 1) = Lstart_t_Lend;

    ros::Time time_end = ros::Time::now();
    std::cout << YELLOW << "Time taken for deskewing: " << time_end.toSec() - time_start.toSec() << " [s]"<< std::endl;
}

Eigen::Vector3d lin_estimator::lincalibManager::deskewPoint(Eigen::Matrix<double, 13, 1> start_point_state,
                                                            Eigen::Matrix<double, 13, 1> current_point_state,
                                                            Eigen::Vector3d skewedPoint,
                                                            Eigen::Matrix3d I_R_L,
                                                            Eigen::Vector3d I_t_L) {

    Eigen::Matrix<double, 4, 1> Istart_q_G = start_point_state.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 3> Istart_R_G = lin_core::quat_2_Rot(Istart_q_G);
    Eigen::Vector3d p_Istart_in_G = start_point_state.block(4, 0, 3, 1);

    Eigen::Matrix<double, 4, 1> Icurr_q_G = current_point_state.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 3> Icurr_R_G = lin_core::quat_2_Rot(Icurr_q_G);
    Eigen::Vector3d p_Icurr_in_G = current_point_state.block(4, 0, 3, 1);

    Eigen::Matrix<double, 3, 3> Istart_R_Icurr = Istart_R_G*Icurr_R_G.transpose();
    Eigen::Vector3d Istart_t_Icurr = Istart_R_G * (p_Icurr_in_G - p_Istart_in_G);

    Eigen::Matrix3d Lstart_R_Lcurr = I_R_L.transpose()*Istart_R_Icurr*I_R_L;
    Eigen::Vector3d Lstart_t_Lcurr = I_R_L.transpose()*Istart_R_Icurr*I_t_L + I_R_L.transpose()*(Istart_t_Icurr - I_t_L);

    Eigen::Vector3d deskewedPoint = Lstart_R_Lcurr * skewedPoint + Lstart_t_Lcurr;

    return deskewedPoint;
}

bool lin_estimator::lincalibManager::try_to_initialize() {

    /// Returns from our initializer
    double time0;
    Eigen::Matrix<double, 4, 1> q_GtoI0;
    Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

    /// Try to initialize the system
    /// We will wait for a jerk
    bool wait_for_jerk = true;
    bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0,v_I0inG,
                                                    b_a0, p_I0inG, wait_for_jerk);

    /// Return if it failed
    if(!success) {
        return false;
    }

    /// Make big vector (q, p, v, bg, ba)
    Eigen::Matrix<double,16,1> imu_val;
    imu_val.block(0,0,4,1) = q_GtoI0;
    imu_val.block(4,0,3,1) << 0,0,0;
    imu_val.block(7,0,3,1) = v_I0inG;
    imu_val.block(10,0,3,1) = b_w0;
    imu_val.block(13,0,3,1) = b_a0;
    state->_imu->set_value(imu_val);
    state->_imu->set_fe(imu_val);
    state->_timestamp = time0;
    startup_time = time0;

    printState();
    return true;
}

void lin_estimator::lincalibManager::feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {
    /// Push into the propagator
    propagator->feed_imu(timestamp, wm, am);

    /// Push into our initializer
    if(!is_initialized_linkalibr) {
        initializer->feed_imu(timestamp, wm, am);
    }
}

void lin_estimator::lincalibManager::feed_measurement_lidar(double timestamp, TPointCloud ::Ptr cloud_raw) {
    if(!is_initialized_linkalibr) {
        is_initialized_linkalibr = try_to_initialize();
        if(!is_initialized_linkalibr)
            return;
    }

    if(params.do_undistortion) {
        TPointCloud::Ptr cloud_undistorted(new TPointCloud);
        Eigen::Matrix4d T_ndt_predict = Eigen::Matrix4d::Identity();
        raw_cloud = *cloud_raw;
        do_undistortion(timestamp, raw_cloud, cloud_undistorted, T_ndt_predict);

        VPointCloud::Ptr cloud_XYZI_undistorted(new VPointCloud);
        TPointCloud2VPointCloud(cloud_undistorted, cloud_XYZI_undistorted);
        undistorted_cloud = *cloud_XYZI_undistorted;

        /// Lidar Odometry
        LOdom->feedScan(timestamp, cloud_XYZI_undistorted, T_ndt_predict);
    } else {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_XYZI(new pcl::PointCloud<pcl::PointXYZI>);
        TPointCloud2VPointCloud(cloud_raw, cloud_XYZI);

        /// Lidar Odometry
        LOdom->feedScan(timestamp, cloud_XYZI);
    }

    /// Propagate and Update
    do_propagate_update(timestamp);

    if(state->_clones_IMU.size() == 1) {
        /// G_T_I1
        Eigen::Matrix<double, 4, 1> q_GtoI1 = state->_imu->quat();
        Eigen::Matrix3d I1_R_G = lin_core::quat_2_Rot(q_GtoI1);
        Eigen::Matrix3d G_R_I1 = I1_R_G.transpose();
        Eigen::Vector3d G_t_I1 = state->_imu->pos();
        G_T_I1.block(0, 0, 3, 3) = G_R_I1;
        G_T_I1.block(0, 3, 3, 1) = G_t_I1;
    }
    /// Printing for debug
    printState();
}


void lin_estimator::lincalibManager::do_propagate_update(double timestamp) {
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "Stepping back in time!!! (prop dt = %3f)\n" RESET, (timestamp-state->_timestamp));
        return;
    }
    /// Propagate the state forward to the current update time
    /// Also augment it with a clone!
    propagator->propagate_and_clone(state, timestamp);
    /// Return if we are unable to propagate
    if (state->_timestamp != timestamp) {
        printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
        printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,timestamp-state->_timestamp);
        return;
    }

    /// Basically getting the timestamp of the first scan in the NDT map
    /// The surfelAssociation code needs this
    if(first_propagation) {
        first_propagation = false;
        map_time = timestamp;
        if(params.gen_data_for_optimization) {
            VPointCloud::Ptr scan_inMap = VPointCloud::Ptr(new VPointCloud);
            pcl::transformPointCloud(undistorted_cloud, *scan_inMap, LOdom->get_current_odometry().pose); /// TODO: Think if lodom should be used or imu odom should be used
            pcl::io::savePCDFileASCII(params.raw_scan_folder_name+"/"+std::to_string(LOdom->get_odom_data().size())+".pcd", raw_cloud);
            pcl::io::savePCDFileASCII(params.deskewed_scan_folder_name+"/"+std::to_string(LOdom->get_odom_data().size())+".pcd", *scan_inMap);
        }
    }
    if(state->_clones_IMU.size() < 2) {
        printf(YELLOW "[lin_estimator::lincalibManager::do_propagate_update] state->_clones_IMU.size() must be > 2\n");
        return;
    }
    /// Marginalize the oldest clone if needed
    if(did_update1 && did_update2) {
        StateHelper::marginalize_old_clone(state);
    }
    relativePose rP = LOdom->get_latest_relativePose();
    updaterLidarOdometry->updateScan2Scan(state, LOdom->get_latest_relativePose(), did_update1);
    Eigen::Matrix4d L1_T_Lk = LOdom->get_current_odometry().pose;
    updaterLidarOdometry->updateScan2GlobalMap(state, L1_T_Lk, G_T_I1, timestamp, did_update2);
    if(did_update1 && did_update2) {
        /// Update our distance traveled
        if(timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
            Eigen::Matrix<double,3,1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
            distance += dx.norm();
        }
        timelastupdate = timestamp;
        if(params.limit_map_size) {
            if(LOdom->get_odom_data().size() < params.no_of_scans_for_map)
                LOdom->append_and_update(true);
            else
                LOdom->append_and_update(false);
        } else {
            LOdom->append_and_update(true);
        }
        if(params.gen_data_for_optimization) {
            VPointCloud::Ptr scan_inMap = VPointCloud::Ptr(new VPointCloud);
            pcl::transformPointCloud(undistorted_cloud, *scan_inMap, LOdom->get_current_odometry().pose);/// TODO: Think if lodom should be used or imu odom should be used
            pcl::io::savePCDFileASCII(params.raw_scan_folder_name+"/"+std::to_string(LOdom->get_odom_data().size())+".pcd", raw_cloud);
            pcl::io::savePCDFileASCII(params.deskewed_scan_folder_name+"/"+std::to_string(LOdom->get_odom_data().size())+".pcd", *scan_inMap);
        }
    }
}

void lin_estimator::lincalibManager::printState() {
//    std::cout << YELLOW << "Started Printing" << std::endl;
    Pose* calib = state->_calib_LIDARtoIMU;

    Eigen::Matrix3d I_R_G = lin_core::quat_2_Rot(state->_imu->quat());
    Eigen::Vector3d G_euler_I = (I_R_G.transpose()).eulerAngles(0, 1, 2);
    double roll = atan2(sin(G_euler_I.x()), cos(G_euler_I.x()))*180/M_PI;
    double pitch = atan2(sin(G_euler_I.y()), cos(G_euler_I.y()))*180/M_PI;
    double yaw = atan2(sin(G_euler_I.z()), cos(G_euler_I.z()))*180/M_PI;

    /// 1
    std::vector<Type*> statevars_pose;
    statevars_pose.push_back(state->_imu->pose());
    Eigen::Matrix<double, 6, 6> covariance_imu_pose = StateHelper::get_marginal_covariance(state, statevars_pose);
    trajfile_csv << ros::Time(state->_timestamp).toNSec() << ", "
                 << state->_imu->quat().x() << ", " << state->_imu->quat().y() << ", "
                 << state->_imu->quat().z() << ", " << state->_imu->quat().w() << ", "
                 << state->_imu->pos().x() << ", " << state->_imu->pos().y() << ", "
                 << state->_imu->pos().z() << ", " << state->_imu->vel().x() << ", "
                 << state->_imu->vel().y() << ", " << state->_imu->vel().z() << std::endl;

    /// 2
    std::vector<Type*> statevars_bias_a;
    statevars_bias_a.push_back(state->_imu->ba());
    Eigen::Matrix<double, 3, 3> covariance_imu_ba = StateHelper::get_marginal_covariance(state, statevars_bias_a);
    std::vector<Type*> statevars_bias_g;
    statevars_bias_g.push_back(state->_imu->bg());
    Eigen::Matrix<double, 3, 3> covariance_imu_bg = StateHelper::get_marginal_covariance(state, statevars_bias_g);
    bias_csv << state->_imu->bias_a().x() << ", " << state->_imu->bias_a().y() << ", " << state->_imu->bias_a().z() << ", "
             << state->_imu->bias_g().x() << ", " << state->_imu->bias_g().y() << ", " << state->_imu->bias_g().z() << ", "
             << sqrt(covariance_imu_ba(0, 0)) << ", " << sqrt(covariance_imu_ba(1, 1)) << ", " << sqrt(covariance_imu_ba(2, 2)) << ", "
             << sqrt(covariance_imu_bg(0, 0)) << ", " << sqrt(covariance_imu_bg(1, 1)) << ", " << sqrt(covariance_imu_bg(2, 2)) << std::endl;

    /// 3
    std::vector<Type*> statevars_velocity;
    statevars_velocity.push_back(state->_imu->v());
    Eigen::Matrix<double, 3, 3> covariance_imu_velocity = StateHelper::get_marginal_covariance(state, statevars_velocity);
    velocity_csv << state->_imu->vel().x() << ", " << state->_imu->vel().y() << ", " << state->_imu->vel().z() << ", "
                 << sqrt(covariance_imu_velocity(0, 0)) << ", "<< sqrt(covariance_imu_velocity(1, 1)) << ", "<< sqrt(covariance_imu_velocity(2, 2)) << std::endl;

    /// 4
    std::vector<Type*> statevars_calib_extrinsic;
    statevars_calib_extrinsic.push_back(state->_calib_LIDARtoIMU);
    Eigen::Matrix<double, 6, 6> covariance_calib_extrinsic = StateHelper::get_marginal_covariance(state, statevars_calib_extrinsic);
    calib_extrinsic_csv << calib->quat()(0) << "," << calib->quat()(1) << ", " << calib->quat()(2) << ", " << calib->quat()(3) << ", "
                        << calib->pos()(0) << "," << calib->pos()(1) << "," << calib->pos()(2) << ", "
                        << sqrt(covariance_calib_extrinsic(0, 0)) << ", " << sqrt(covariance_calib_extrinsic(1, 1)) << ", " << sqrt(covariance_calib_extrinsic(2, 2)) << ", "
                        << sqrt(covariance_calib_extrinsic(3, 3)) << ", " << sqrt(covariance_calib_extrinsic(4, 4)) << ", " << sqrt(covariance_calib_extrinsic(5, 5)) << std::endl;

    /// 5
    std::vector<Type*> statevars_calib_dt;
    statevars_calib_dt.push_back(state->_calib_dt_LIDARtoIMU);
    Eigen::Matrix<double, 1, 1> covariance_calib_dt = StateHelper::get_marginal_covariance(state, statevars_calib_dt);
    calib_dt_csv << state->_calib_dt_LIDARtoIMU->value()(0) << ", " << sqrt(covariance_calib_dt(0, 0)) << std::endl;
//    std::cout << GREEN << "Time Delay: " << 1000*state->_calib_dt_LIDARtoIMU->value()(0) << " [ms]" << std::endl;
//    std::cout << YELLOW << "Done Printing" << std::endl;
}

