//
// Created by usl on 11/6/20.
//

#ifndef LINCALIB_STATEOPTIONS_H
#define LINCALIB_STATEOPTIONS_H

#include <types/Type.h>

using namespace lin_type;

namespace lin_estimator {
    /// Struct which stores all filter options
    struct StateOptions {
        /// Bool to determine whether or not to do First Estimate Jacobians
        bool do_fej = true;

        /// Bool to determine whether or not to use imu message averaging
        bool imu_avg = false;

        /// Bool to determine whether we want to use Runge-Kutta 4 IMU integration
        bool use_rk4_integration = true;

        /// Bool to determine whether or not to calibrate Lidar to IMU time offset
        bool do_calib_lidar_timeoffset = false;

        /// Max clone size of sliding window
        int max_clone_size = 11;

        /// Bool to determine whether or not to calibrate LIDAR to IMU (I_R_L (I_q_L), I_t_L) extrinsics
        bool do_calib_extrinsic = false;

        /// Initial Noise
        double rot_x_noise = 0.2;
        double rot_y_noise = 0.2;
        double rot_z_noise = 0.2;
        double trans_x_noise = 0.3;
        double trans_y_noise = 0.3;
        double trans_z_noise = 0.3;
        double time_offset_noise = 0.01;

        /// Print function of what parameters we have loaded
        void print() {
            printf("\t- use_fej: %d\n", do_fej);
            printf("\t- use_imuavg: %d\n", imu_avg);
            printf("\t- use_rk4int: %d\n", use_rk4_integration);
            printf("\t- do_calib_lidar_timeoffset?: %d\n", do_calib_lidar_timeoffset);
            printf("\t- max_clone_size: %d\n", max_clone_size);
            printf("\t- do_calib_extrinsic?: %d\n", do_calib_extrinsic);
            printf("\t- rot_x_noise: %f\n", rot_x_noise);
            printf("\t- rot_y_noise: %f\n", rot_y_noise);
            printf("\t- rot_z_noise: %f\n", rot_z_noise);
            printf("\t- trans_x_noise: %f\n", trans_x_noise);
            printf("\t- trans_y_noise: %f\n", trans_y_noise);
            printf("\t- trans_z_noise: %f\n", trans_z_noise);
            printf("\t- time_offset_noise: %f\n", time_offset_noise);
        }
    };
};
#endif //LINCALIB_STATEOPTIONS_H
