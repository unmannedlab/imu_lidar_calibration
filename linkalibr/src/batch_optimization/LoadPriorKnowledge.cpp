//
// Created by usl on 3/12/21.
//

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <batch_optimization/LoadPriorKnowledge.h>

#include <boost/tokenizer.hpp>

namespace lin_estimator {

    Eigen::Matrix4d LoadPriorKnowledge::GetTransformationMatrix(std::string filename) {
        std::ifstream initial_calib(filename);
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
        return T;
    }

    void LoadPriorKnowledge::LoadIMUMeasurements() {
        std::ifstream imu_measurements_csv(params_.imu_measurements_csv_filename);
        if(!imu_measurements_csv.is_open()) {
            return;
        }
        typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
        std::vector< std::string > vec;
        std::string line;
        while(getline(imu_measurements_csv,line)) {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());
            if(vec.size()!=7) {
                ROS_ERROR_STREAM("Vec Size not consistent at [LoadIMUMeasurements]");
                ros::shutdown();
            }
            uint64_t imu_timestamp = std::stoll(*vec.begin());
            double ax = std::stod(*(vec.begin()+1));
            double ay = std::stod(*(vec.begin()+2));
            double az = std::stod(*(vec.begin()+3));
            double wx = std::stod(*(vec.begin()+4));
            double wy = std::stod(*(vec.begin()+5));
            double wz = std::stod(*(vec.begin()+6));
            ImuMeasurement imu_measurement;
            imu_measurement.timestamp = imu_timestamp;
            imu_measurement.am = Eigen::Vector3d(ax, ay, az);
            imu_measurement.wm = Eigen::Vector3d(wx, wy, wz);
            imu_measurements_.emplace_back(imu_measurement);
        }
        std::cout << "No of imu measurements: " << imu_measurements_.size() << std::endl;
    }

    void LoadPriorKnowledge::LoadIMUTrajectory() {
        std::cout << "LoadPriorKnowledge::LoadIMUTrajectory: " << params_.imu_trajectory_csv_filename << std::endl;
        std::ifstream imu_trajectory_csv(params_.imu_trajectory_csv_filename);
        if(!imu_trajectory_csv.is_open()) {
            return;
        }
        typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
        std::vector< std::string > vec;
        std::string line;
        int i = 0;
        while(getline(imu_trajectory_csv, line)) {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());
            if(vec.size()!=8) {
                ROS_ERROR_STREAM("Vec Size not consistent at [LoadIMUTrajectory]");
                ros::shutdown();
            }
            uint64_t lidar_trajectory_timestamp = std::stoll(*vec.begin());
            double qx = std::stod(*(vec.begin()+1));
            double qy = std::stod(*(vec.begin()+2));
            double qz = std::stod(*(vec.begin()+3));
            double qw = std::stod(*(vec.begin()+4));
            double x = std::stod(*(vec.begin()+5));
            double y = std::stod(*(vec.begin()+6));
            double z = std::stod(*(vec.begin()+7));
//            double vx = std::stod(*(vec.begin()+8));
//            double vy = std::stod(*(vec.begin()+9));
//            double vz = std::stod(*(vec.begin()+10));

            Eigen::Quaterniond orientation;
            orientation.x() = qx; orientation.y() = qy; orientation.z() = qz; orientation.w() = qw;
            Eigen::Vector3d position;
            position.x() = x; position.y() = y; position.z() = z;
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
//            velocity.x() = vx; velocity.y() = vy; velocity.z() = vz;
            PoseVel imu_pose_vel;
            imu_pose_vel.timestamp = lidar_trajectory_timestamp;
            imu_pose_vel.orientation = orientation;
            imu_pose_vel.position = position;
            imu_pose_vel.velocity = velocity;
            if(!imu_trajectory_.insert(std::make_pair(size_t(i), imu_pose_vel)).second)
                std::cout << "Could not insert at: " << i << std::endl;
            i++;
        }
        assert(i == imu_trajectory_.size());
        std::cout << "No of elements in imu trajectory container: " << imu_trajectory_.size() << std::endl;
    }

    void LoadPriorKnowledge::LoadPlaneParams() {
        std::ifstream plane_params_csv(params_.plane_params_csv_filename);
        if(!plane_params_csv.is_open()) {
            return;
        }
        typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
        std::vector< std::string > vec;
        std::string line;
        while(getline(plane_params_csv,line)) {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());
            if(vec.size()!=4) {
                ROS_ERROR_STREAM("Vec Size not consistent at [LoadPlaneParams]");
                ros::shutdown();
            }
            double x = std::stod(*vec.begin());
            double y = std::stod(*(vec.begin()+1));
            double z = std::stod(*(vec.begin()+2));
            double w = std::stod(*(vec.begin()+3));
            Eigen::Vector4d plane_param(x, y, z, w);
            plane_params_.emplace_back(plane_param);
        }
        std::cout << "No of elements plane_params_ container: " << plane_params_.size() << std::endl;
    }

    void LoadPriorKnowledge::LoadPlanarPoints() {
        std::ifstream planar_points_csv(params_.planar_points_csv_filename);
        if(!planar_points_csv.is_open()) {
            return;
        }
        typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
        std::vector< std::string > vec;
        std::string line;
        while (getline(planar_points_csv,line)) {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());
            if(vec.size() != 7) {
                ROS_ERROR_STREAM("Vec Size not consistent at [LoadPlanarPoints]");
                ros::shutdown();
            }
            PlanarPoint planar_point;
            uint64_t timestamp_scan = std::stoull(*vec.begin());
            planar_point.timestamp_scan = timestamp_scan;
            uint64_t timestamp_point = std::stoull(*(vec.begin()+1));
            planar_point.timestamp_point = timestamp_point;
            int scan_id = std::stoi(*(vec.begin()+2));
            planar_point.scan_id = scan_id;
            int plane_id = std::stoi(*(vec.begin()+3));
            planar_point.plane_id = plane_id;
            Eigen::Vector3d lidar_point = Eigen::Vector3d(std::stod(*(vec.begin()+4)),
                                                          std::stod(*(vec.begin()+5)),
                                                          std::stod(*(vec.begin()+6)));
            planar_point.point = lidar_point;
            planar_points_.emplace_back(planar_point);
        }
        std::cout << "No of elements in planar_points_ container: " << planar_points_.size() << std::endl;
    }
}