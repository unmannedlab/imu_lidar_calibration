//
// Created by usl on 11/8/20.
//

#ifndef LINCALIB_STATEHELPER_H
#define LINCALIB_STATEHELPER_H

#include "State.h"
#include "utils/color.h"

#include <boost/math/distributions/chi_squared.hpp>

using namespace lin_core;

namespace lin_estimator {
    /// This is a helper function which manipulates the State and its covariance
    /// This class has all functions that change the covariance along with addition and removing elements from the state.
    /// All functions are static.
    class StateHelper {
    public:
        /// Performs EKF propagation of the state covariance
        static void EKFPropagation(State *state,  /// Pointer to the state
                                   const std::vector<Type*> &order_NEW, /// Contiguous variables that have evolved according to this state transition
                                   const std::vector<Type*> &order_OLD, /// Variable ordering used in the state transition
                                   const Eigen::MatrixXd &Phi, /// State transition matrix
                                   const Eigen::MatrixXd &Q); /// Additive state propagation noise matrix

        /// Performs EKF update of the state
        static void EKFUpdate(State *state, /// Pointer to the state
                              const std::vector<Type *> &H_order, /// Variable ordering used in compressed Jacobian
                              const Eigen::MatrixXd &H, /// Condensed Jacobian of updating measurement
                              const Eigen::VectorXd &res,  /// residual of updating measurement
                              const Eigen::MatrixXd &R); /// updating measurment covariance

        /// The following will calculate a smaller covariance for a given set of variables
        static Eigen::MatrixXd get_marginal_covariance(State *state, /// Pointer to state
                                                       const std::vector<Type *> &small_variables); /// Vector of variables whose marginal covariance is desired

        /// The following will calculate the full covariance matrix
        static Eigen::MatrixXd get_full_covariance(State *state);

        /// Marginalize
        static void marginalize(State *state, /// Pointer to state
                                Type *marginalize); /// Pointer to variable to marginalize

        /// Clones "variables to clone" and places it at the end of covariance
        static Type* clone (State *state, /// Pointer to state
                            Type *variable_to_clone); /// Pointer to variable that will be cloned

        /// Initializes a new variable into covariance
        static bool initialize(State *state, ///  Pointer to state
                               Type *new_variable, /// Pointer to variable to be initialized
                               const std::vector<Type *> &H_order, /// Vector of pointers in order they are contained in the condensed state Jacobian
                               Eigen::MatrixXd &H_R, /// Jacobian of initializing measurements wrt variables in H_order
                               Eigen::MatrixXd &H_L, /// Jacobian of initializing measurments wrt new variable
                               Eigen::MatrixXd &R, /// Covariance of initializing measurements
                               Eigen::VectorXd &res, /// Residual of initializing measurements
                               double chi_2_mult);  /// Value we should multiply the chi2 threshold by (larger means it will be accepting more measurements)

         /// Initializes a new variable into covariance (H_L must be invertible)
         static void initialize_invertible(State *state,
                                           Type *new_variable,
                                           const std::vector<Type *> &H_order,
                                           const Eigen::MatrixXd &H_R,
                                           const Eigen::MatrixXd &H_L,
                                           const Eigen::MatrixXd &R,
                                           const Eigen::VectorXd &res);

        /// Augment the state with a stochastic copy of the current IMU pose
        static void augment_clone(State *state, /// Pointer to state
                                  Eigen::Matrix<double, 3, 1> last_w); /// The estimated angular velocity at the cloning time
                                                                       /// can be used to estimate the imu-lidar time offset
        /// Remove the oldest clone, if we have more than the max clone count
        /// This will marginalize the clone from our covariance and remove it from our state.
        static void marginalize_old_clone(State *state) {
            if ((int) state->_clones_IMU.size() > state->_options.max_clone_size) {
                double marginal_time = state->margtimestep();
                assert(marginal_time != INFINITY);
                StateHelper::marginalize(state, state->_clones_IMU.at(marginal_time));
                /// Note that the marginalizer should have already deleted the clone
                /// Thus we just need to remove the pointer to it from our state
                state->_clones_IMU.erase(marginal_time);
            }
        }

    private:
        /// All function in this class should be static
        /// Thus an instance of this class cannot be created
        StateHelper() {}
    };
}
#endif //LINCALIB_STATEHELPER_H
