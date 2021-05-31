//
// Created by usl on 3/12/21.
//

#ifndef LINKALIBR_LOADPRIORKNOWLEDGE_H
#define LINKALIBR_LOADPRIORKNOWLEDGE_H

#include "ros/ros.h"

#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utils/eigen_utils.h>

namespace lin_estimator {
    class OptimizerParams {
    public:
        std::string imu_measurements_csv_filename;
        std::string imu_trajectory_csv_filename;
        std::string imu_trajectory_out_csv_filename;
        std::string plane_params_csv_filename;
        std::string planar_points_csv_filename;
        std::string planar_points_deskewed_csv_filename;
        std::string initial_calib_filename;
        std::string init_calib_csv_filename;
        std::string final_calib_csv_filename;
        std::string imu_velocity_out_csv_filename;

        double accelerometer_noise_density;
        double accelerometer_random_walk;
        double gyroscope_noise_density;
        double gyroscope_random_walk;
    };

    struct PlanarPoint {
        uint64_t timestamp_scan;
        uint64_t timestamp_point;
        size_t scan_id;
        size_t plane_id;
        Eigen::Vector3d point;
    };

    struct ImuMeasurement {
        uint64_t timestamp;
        Eigen::Vector3d am;
        Eigen::Vector3d wm;
    };

    struct PoseVel {
        uint64_t timestamp;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
    };

    class LoadPriorKnowledge {

    private:
        void LoadIMUMeasurements();
        void LoadIMUTrajectory();
        void LoadPlaneParams();
        void LoadPlanarPoints();
        Eigen::Matrix4d GetTransformationMatrix(std::string filename);

    public:
        LoadPriorKnowledge() {
            /// Default Constructor
        }

        LoadPriorKnowledge(OptimizerParams params) {
            params_ = params;
            I_T_L = GetTransformationMatrix(params.initial_calib_filename);

            LoadIMUMeasurements();
            LoadIMUTrajectory();
            LoadPlaneParams();
            LoadPlanarPoints();
        }

        void GetIMUMeasurements(Eigen::aligned_vector<ImuMeasurement>& _imu_measurements) {
            _imu_measurements = imu_measurements_;
        }

        void GetIMUTrajectory(std::map<size_t , PoseVel>& _imu_trajectory) {
            _imu_trajectory = imu_trajectory_;
        }

        void GetPlaneParams(Eigen::aligned_vector<Eigen::Vector4d>& _plane_params) {
            _plane_params = plane_params_;
        }

        void GetPlanarPoints(Eigen::aligned_vector<PlanarPoint>& _planar_points) {
            _planar_points = planar_points_;
        }

        void GetInitialCalib(Eigen::Matrix4d& _I_T_L) {
            _I_T_L = I_T_L;
        }

    private:
        OptimizerParams params_;
        Eigen::aligned_vector<ImuMeasurement> imu_measurements_;
        std::map<size_t, PoseVel> imu_trajectory_;
        Eigen::aligned_vector<Eigen::Vector4d> plane_params_;
        Eigen::aligned_vector<PlanarPoint> planar_points_;
        Eigen::Matrix4d I_T_L;
    };
}


#endif //LINKALIBR_LOADPRIORKNOWLEDGE_H
