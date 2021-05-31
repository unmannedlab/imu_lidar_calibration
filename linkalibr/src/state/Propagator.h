//
// Created by usl on 11/6/20.
//

#ifndef LINCALIB_PROPAGATOR_H
#define LINCALIB_PROPAGATOR_H

#include "State.h"
#include "StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/color.h"

using namespace lin_core;
namespace lin_estimator {
    /// Performs the state covariance and mean propagation using IMU measurement
    class Propagator {
    public:
        /// Struct for a single IMU measurement (time, w_m, a_m)
        struct IMUDATA {
            /// Timestamp of the reading
            double timestamp;

            /// Gyroscope reading, angular velocity (rad/s)
            Eigen::Matrix<double, 3, 1> wm;

            /// Accelerometer reading, linear acceleration (m/s^2)
            Eigen::Matrix<double, 3, 1> am;
        };

        /// Struct of imu noise parameters
        struct NoiseManager {
            /// Gyroscope white noise (rad/s/sqrt(hz))
            double sigma_w = 1.6968e-04;
            /// Gyroscope random walk (rad/s^2/sqrt(hz))
            double sigma_wb = 1.9393e-05;
            /// Accelerometer white noise (m/s^2/sqrt(hz))
            double sigma_a = 2.0000e-3;
            /// Accelerometer random walk (m/s^3/sqrt(hz))
            double sigma_ab = 3.0000e-03;
            /// Print function of noise parameters loaded
            void print() {
                printf("\t- gyroscope_noise white noise: %.6f\n", sigma_w);
                printf("\t- accelerometer_noise white noise: %.5f\n", sigma_a);
                printf("\t- gyroscope_random_walk noise: %.7f\n", sigma_wb);
                printf("\t- accelerometer_random_walk noise: %.7f\n", sigma_ab);
            }
        };
        /// Default constructor
        Propagator(NoiseManager noises, Eigen::Vector3d gravity) {
            _noises = noises;
            _gravity = gravity;
            last_prop_time_offset = 0.0;
        }
        /// Stores incoming inertial readings
        void feed_imu(double timestamp, /// timestamp of any imu reading
                      Eigen::Vector3d wm, /// Gyro angular reading
                      Eigen::Vector3d am) /// Accelerometer reading
        {
            /// Create our imu data object
            IMUDATA data;
            data.timestamp = timestamp;
            data.wm = wm;
            data.am = am;

            /// Append it to our vector
            imu_data.push_back(data);

            /// Loop through and delete IMU messages that are older than 20 seconds
            auto it0 = imu_data.begin();
            while(it0 != imu_data.end()) {
                if(timestamp-(*it0).timestamp > 20)
                    it0 = imu_data.erase(it0);
                else
                    it0++;
            }
        }
        /// Propagate state up to given timestamp and then clone
        /**
        * This will first collect all imu readings that occured between the
        * *current* state time and the new time we want the state to be at.
        * If we don't have any imu readings we will try to extrapolate into the future.
        * After propagating the mean and covariance using our dynamics,
        * We clone the current imu pose as a new clone in our state.
        */
        void propagate_and_clone(State *state, /// Pointer to state
                                 double timestamp); /// Time to propagate to and clone at
        /// Gets what the state and its covariance will be at a given timestamp
        /**
        * This can be used to find what the state will be in the "future" without propagating it.
        * This will propagate a clone of the current IMU state and its covariance matrix.
        * This is typically used to provide high frequency pose estimates between updates.
        */
        void fast_state_propagate(State *state, /// Pointer to state
                                  double timestamp, /// Time to propagate to
                                  Eigen::Matrix<double, 13, 1> &state_plus); /// The propagated state (q_GtoI, p_IinG, v_IinG, w_IinI)

         void fast_state_propagate(Eigen::Matrix<double, 10, 1> state,
                                   Eigen::Matrix<double, 3, 1> bg,
                                   Eigen::Matrix<double, 3, 1> ba,
                                   Eigen::Matrix<double, 10, 1>& state_plus,
                                   double time_0,
                                   double time_1);

        /// Function that given current imu_data, will select imu readings between two times
        /**
        * This will create measurements that we will integrate with, and an extra measurement at the end.
        * We use the @ref interpolate_data() function to "cut" the imu readings at the begining and end of the integration.
        * The timestamps passed should already take into account the time offset values.
        */
        static std::vector<IMUDATA> select_IMU_readings(const std::vector<IMUDATA>& imu_data, /// IMU data we will select measurements from
                                                        double time0, /// Start timestamp
                                                        double time1); /// End timestamp
        /// Helper function that will linearly interpolate IMU readings
        /**
        * This should be used instead of just "cutting" imu messages that bound the camera times
        * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
        */
        static IMUDATA interpolate_data(const IMUDATA imu_1, /// imu at begining of interpolation interval
                                        const IMUDATA imu_2, /// imu at end of interpolation interval
                                        double timestamp) /// Timestamp being interpolated to
        {
            /// time-distant lambda
            double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
            /// interpolate between the two times
            IMUDATA data;
            data.timestamp = timestamp;
            data.am = (1-lambda) * imu_1.am + lambda * imu_2.am;
            data.wm = (1-lambda) * imu_1.wm + lambda * imu_2.wm;
            return data;
        }

        std::vector<IMUDATA> get_imu_data() {
            return imu_data;
        }

    protected:
        /// Estimate for time offset at last propagation time
        double last_prop_time_offset = -INFINITY;
        bool have_last_prop_time_offset = false;
        /// Propagates the state forward using the IMU data and computes the noise covariance and state transition
        void predict_and_compute(State *state, /// Pointer to state
                                 const IMUDATA data_minus, /// IMU readings at the beginning of interval
                                 const IMUDATA data_plus, /// IMU readings at the end of the interval
                                 Eigen::Matrix<double, 15, 15> &F, /// State-transition matrix over the interval
                                 Eigen::Matrix<double, 15, 15> &Qd); /// Discrete-time noise
        /// Discrete imu mean propagation
        void predict_mean_discrete(State *state, /// Pointer to state
                                   double dt, /// Time we should integrate over
                                   const Eigen::Vector3d &w_hat1, /// Angular velocity with bias removed
                                   const Eigen::Vector3d &a_hat1, /// Linear acceleration with bias removed
                                   const Eigen::Vector3d &w_hat2, /// Next angular velocity with bias removed
                                   const Eigen::Vector3d &a_hat2, /// Next linear acceleration with bias removed
                                   Eigen::Vector4d &new_q, /// The resulting new orientation after integration
                                   Eigen::Vector3d &new_v, /// The resulting new velocity after integration
                                   Eigen::Vector3d &new_p);/// The resulting new position after integration
        /// Runga Kutta 4 imu mean propagation
        void predict_mean_rk4(State *state, /// Pointer to state
                              double dt, /// Time we should integrate over
                              const Eigen::Vector3d &w_hat1, /// Angular velocity with bias removed
                              const Eigen::Vector3d &a_hat1, /// Linear acceleration with bias removed
                              const Eigen::Vector3d &w_hat2, /// Next angular velocity with bias removed
                              const Eigen::Vector3d &a_hat2, /// Next linear acceleration with bias removed
                              Eigen::Vector4d &new_q, /// The resulting new orientation after integration
                              Eigen::Vector3d &new_v, /// The resulting new velocity after integration
                              Eigen::Vector3d &new_p);/// The resulting new position after integration
        /// Container for the noise values
        NoiseManager _noises;
        /// Our history of IMU messages (time, angular, linear)
        std::vector<IMUDATA> imu_data;
        /// Gravity vector
        Eigen::Matrix<double, 3, 1> _gravity;
    };
};
#endif //LINCALIB_PROPAGATOR_H

