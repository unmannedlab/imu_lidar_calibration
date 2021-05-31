//
// Created by usl on 11/7/20.
//

#include "Propagator.h"

using namespace lin_core;
using namespace lin_estimator;

std::vector<Propagator::IMUDATA> Propagator::select_IMU_readings(const std::vector<IMUDATA> &imu_data, /// IMU data we will select measurements from
                                                                 double time0, /// Start timestamp
                                                                 double time1) { /// End timestamp
    /// Vector IMU Readings
    std::vector<Propagator::IMUDATA> prop_data;
    /// Ensure we have some measurements
    if(imu_data.empty()) {
        printf(YELLOW "Propagator::select_IMU_readings(): No IMU measurements.\n" RESET);
        return prop_data;
    }
    /// Loop through and find all the needed measurements to propagate with
    /// The measurement is split based on the given state time and the update timestamp

    for (size_t i = 0; i < imu_data.size()-1; i++) {
        /// Start of the integration period

        /// If the next timestamp is greater than our current state time
        /// And the current is not greater than it
        /// Then we should split our current IMU measurement
        /// The cond below is t_{m_i} < t_{s} < t_{m_{i+1}}
        /// Here we determine the imu measurement corresponding to the start of the integration time stamp i.e. time0
        if (imu_data.at(i).timestamp < time0 && time0 < imu_data.at(i+1).timestamp) {
            IMUDATA data = Propagator::interpolate_data(imu_data.at(i), imu_data.at(i+1), time0);
            prop_data.push_back(data);
            continue;
        }

        /// Middle of the integration period
        /// If our IMU measurement is in between our propagation period
        /// Then we should just append the whole measurement time to our propagation vector
        if(time0 <= imu_data.at(i).timestamp && imu_data.at(i+1).timestamp <= time1) {
            prop_data.push_back(imu_data.at(i));
            continue;
        }

        /// End of the integration period
        /// If the current timestamp is greater than the update time
        /// We should split the next IMU measurement to the update time
        if (imu_data.at(i+1).timestamp > time1) {
            if (imu_data.at(i).timestamp > time1) {
                IMUDATA data = interpolate_data(imu_data.at(i-1), imu_data.at(i), time1);
                prop_data.push_back(data);
            } else {
                prop_data.push_back(imu_data.at(i));
            }
            if(prop_data.at(prop_data.size()-1).timestamp != time1) {
                IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), time1);
                prop_data.push_back(data);
            }
            break;
        }
    }

    /// Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        printf(YELLOW "Propagator::select_imu_readings(): No IMU measurement to propgate with (%d of 2) \n"
               RESET, (int)prop_data.size());
        return prop_data;
    }

    /// Loop through and ensure we do not have an zero dt values
    /// This would cause the noise covariance to be Infinity
    for (size_t i=0; i < prop_data.size()-1; i++) {
        if (std::abs(prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) < 1e-12) {
            printf(YELLOW "Propagator::select_imu_readings(): Zero DT between IMU reading %d and %d, removing it!\n" RESET, (int)i, (int)(i+1));
            prop_data.erase(prop_data.begin()+i);
            i--;
        }
    }

    /// Check that we have at least one measurement to propagate with
    if(prop_data.size() < 2) {
        printf(YELLOW "Propagator::select_imu_readings(): No IMU measurements to propagate with (%d of 2).\n" RESET, (int)prop_data.size());
        return prop_data;
    }

    return prop_data;
}

void Propagator::fast_state_propagate(State *state, /// Pointer to state
                                      double timestamp, /// Time to propagate to
                                      Eigen::Matrix<double, 13, 1> &state_plus) /// The propagated state (q_GtoI, p_IinG, v_IinG, w_IinI)
{
    /// Set the last time offset value if we have just started the system up
    if(!have_last_prop_time_offset) {
        last_prop_time_offset = state->_calib_dt_LIDARtoIMU->value()(0);
        have_last_prop_time_offset = true;
    }

    /// Get what our IMU-LIDAR offset should be (t_imu = t_lidar + calib_dt)
    double t_off_new = state->_calib_dt_LIDARtoIMU->value()(0);

    /// First lets construct an IMU vector of measurements we need
    double time0 = state->_timestamp+last_prop_time_offset;
    double time1 = timestamp+t_off_new;
    vector<IMUDATA> prop_data = Propagator::select_IMU_readings(imu_data, time0, time1);
    /// Save the original IMU state
    Eigen::VectorXd orig_val = state->_imu->value();
    Eigen::VectorXd orig_fe = state->_imu->fe();

    /// Loop through all IMU messages, and use them to move the state forward in time
    /// This uses the zero'th order quat, and then constant acceleration discrete
    if(prop_data.size() > 1) {
        for(size_t i=0; i<prop_data.size()-1; i++) {
            /// Time elapsed over interval
            double dt = prop_data.at(i+1).timestamp-prop_data.at(i).timestamp;
            //assert(data_plus.timestamp>data_minus.timestamp);
            /// Corrected imu measurements
            Eigen::Matrix<double,3,1> w_hat1 = prop_data.at(i).wm - state->_imu->bias_g();
            Eigen::Matrix<double,3,1> a_hat1 = prop_data.at(i).am - state->_imu->bias_a();
            Eigen::Matrix<double,3,1> w_hat2 = prop_data.at(i+1).wm - state->_imu->bias_g();
            Eigen::Matrix<double,3,1> a_hat2 = prop_data.at(i+1).am - state->_imu->bias_a();
            /// Compute the new state mean value
            Eigen::Vector4d new_q;
            Eigen::Vector3d new_v, new_p;
            if(state->_options.use_rk4_integration)
                predict_mean_rk4(state, dt, w_hat1, a_hat1, w_hat2, a_hat2, new_q, new_v, new_p);
            else
                predict_mean_discrete(state, dt, w_hat1, a_hat1, w_hat2, a_hat2, new_q, new_v, new_p);
            /// Now replace imu estimate and fej with propagated values
            Eigen::Matrix<double,16,1> imu_x = state->_imu->value();
            imu_x.block(0,0,4,1) = new_q;
            imu_x.block(4,0,3,1) = new_p;
            imu_x.block(7,0,3,1) = new_v;
            state->_imu->set_value(imu_x);
            state->_imu->set_fe(imu_x);
        }
    }
    /// Now record what the predicted state should be
    state_plus = Eigen::Matrix<double,13,1>::Zero();
    state_plus.block(0,0,4,1) = state->_imu->quat();
    state_plus.block(4,0,3,1) = state->_imu->pos();
    state_plus.block(7,0,3,1) = state->_imu->vel();
    if(prop_data.size() > 1)
        state_plus.block(10,0,3,1) = prop_data.at(prop_data.size()-2).wm - state->_imu->bias_g();
    else if(!prop_data.empty())
        state_plus.block(10,0,3,1) = prop_data.at(prop_data.size()-1).wm - state->_imu->bias_g();
    /// Finally replace the imu with the original state we had
    state->_imu->set_value(orig_val);
    state->_imu->set_fe(orig_fe);
}

void Propagator::fast_state_propagate(Eigen::Matrix<double, 10, 1> state,
                                      Eigen::Matrix<double, 3, 1> bg,
                                      Eigen::Matrix<double, 3, 1> ba,
                                      Eigen::Matrix<double, 10, 1>& state_plus,
                                      double time_0,
                                      double time_1) {
    /// TODO: We need to take care of time offset
    /// First lets construct an IMU vector of measurements we need
    vector<IMUDATA> prop_data = Propagator::select_IMU_readings(imu_data, time_0, time_1);
//    std::cout << "Prop Data Size: " << prop_data.size() << std::endl;
    /// Loop through all IMU messages, and use them to move the state forward in time
    /// This uses the zero'th order quat, and then constant acceleration discrete
    if(prop_data.size() > 1) {
        for(size_t i=0; i<prop_data.size()-1; i++) {
            /// Time elapsed over interval
            double dt = prop_data.at(i+1).timestamp-prop_data.at(i).timestamp;
            //assert(data_plus.timestamp>data_minus.timestamp);
            /// Corrected imu measurements
            Eigen::Matrix<double,3,1> w_hat1 = prop_data.at(i).wm - bg;
            Eigen::Matrix<double,3,1> a_hat1 = prop_data.at(i).am - ba;
            Eigen::Matrix<double,3,1> w_hat2 = prop_data.at(i+1).wm - bg;
            Eigen::Matrix<double,3,1> a_hat2 = prop_data.at(i+1).am - ba;

            // If we are averaging the IMU, then do so
//            Eigen::Vector3d w_hat = w_hat1;
//            Eigen::Vector3d a_hat = a_hat1;

            Eigen::Vector3d w_hat = .5*(w_hat1+w_hat2);
            Eigen::Vector3d a_hat = .5*(a_hat1+a_hat2);

            Eigen::Vector4d quat_GtoI = state.block(0, 0, 4, 1);
            Eigen::Vector3d pos_IinG = state.block(4, 0, 3, 1);
            Eigen::Vector3d vel_IinG = state.block(7, 0, 3, 1);

            /// Compute the new state mean value
            Eigen::Vector4d new_q;
            Eigen::Vector3d new_v;
            Eigen::Vector3d new_p;

            // Pre-compute things
            double w_norm = w_hat.norm();
            Eigen::Matrix<double,4,4> I_4x4 = Eigen::Matrix<double,4,4>::Identity();
            Eigen::Matrix<double,3,3> R_GtoI = lin_core::quat_2_Rot(quat_GtoI);

            // Orientation: Equation (101) and (103) and of Trawny indirect TR
            Eigen::Matrix<double,4,4> bigO;
            if(w_norm > 1e-20) {
                bigO = cos(0.5*w_norm*dt)*I_4x4 + 1/w_norm*sin(0.5*w_norm*dt)*Omega(w_hat);
            } else {
                bigO = I_4x4 + 0.5*dt*Omega(w_hat);
            }

            // Rotation
            new_q = quatnorm(bigO*quat_GtoI);
            // Velocity: just the acceleration in the local frame, minus global gravity
            new_v = vel_IinG + R_GtoI.transpose()*a_hat*dt - _gravity*dt;
            // Position: just velocity times dt, with the acceleration integrated twice
            new_p = pos_IinG + vel_IinG*dt + 0.5*R_GtoI.transpose()*a_hat*dt*dt - 0.5*_gravity*dt*dt;

            state.block(0, 0, 4, 1) = new_q;
            state.block(4, 0, 3, 1) = new_p;
            state.block(7, 0, 3, 1) = new_v;
        }
    }
    state_plus = state;
}

void Propagator::propagate_and_clone(State *state, /// Pointer to state
                                     double timestamp) /// Time to propagate to and clone at
{
    /// If the difference between the current update time and state is zero
    /// We should crash as this means we would have two clones at the same time
    if (state->_timestamp == timestamp) {
        printf(RED "Propagator::propagate_and_clone(): "
                   "Propagation called again at same timestep at last update timestep!!!!\\n" RESET);
        std::exit(EXIT_FAILURE);
    }
    /// We should crash if we are trying to propagate backwards
    if(state->_timestamp > timestamp) {
        printf(RED "Propagator::propagate_and_clone(): Propagation called trying to propagate backwards in time!!!!\n" RESET);
        printf(RED "Propagator::propagate_and_clone(): desired propagation = %.4f\n" RESET, (timestamp-state->_timestamp));
        std::exit(EXIT_FAILURE);
    }
    /// Set the last time offset value if we have just started the system up
    if(!have_last_prop_time_offset) {
        last_prop_time_offset = state->_calib_dt_LIDARtoIMU->value()(0);
        have_last_prop_time_offset = true;
    }
    /// Get what IMU-Lidar time offset should be
    double t_off_new = state->_calib_dt_LIDARtoIMU->value()(0);
    /// First lets construct an IMU vector of measurements we need
    double time0 = state->_timestamp + last_prop_time_offset;
    double time1 = timestamp + t_off_new;
    std::vector<IMUDATA> prop_data = Propagator::select_IMU_readings(imu_data, time0, time1);
    /// We are going to sum up all the state transition matrices, so we can do a single large multiplication at the end
    /// Phi_summed = Phi_i*Phi_summed
    /// Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
    /// After summing we can multiply the total phi to get the updated covariance
    /// We will then add the noise to the IMU portion of the state
    Eigen::Matrix<double,15,15> Phi_summed = Eigen::Matrix<double,15,15>::Identity();
    Eigen::Matrix<double,15,15> Qd_summed = Eigen::Matrix<double,15,15>::Zero(); /// Set to zero initially
    double dt_summed = 0;

    /// Loop through all IMU messages, and use them to move the state forward in time
    /// This uses the zero'th order quat, and then constant acceleration discrete
    if(prop_data.size() > 1) {
        for(size_t i=0; i<prop_data.size()-1; i++) {

            /// Get the next state Jacobian and noise Jacobian for this IMU reading
            Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
            Eigen::Matrix<double, 15, 15> Qdi = Eigen::Matrix<double, 15, 15>::Zero();
            predict_and_compute(state, prop_data.at(i), prop_data.at(i+1), F, Qdi);

            /// -Next we should propagate our IMU covariance
            /// Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
            /// Pci' = F*Pci and Pic' = Pic*F.transpose()
            /// NOTE: Here we are summing the state transition F so we can do a single mutiplication later
            /// NOTE: Phi_summed = Phi_i*Phi_summed
            /// NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
            Phi_summed = F * Phi_summed;
            Qd_summed = F * Qd_summed * F.transpose() + Qdi;
            Qd_summed = 0.5*(Qd_summed+Qd_summed.transpose());
            dt_summed += prop_data.at(i+1).timestamp-prop_data.at(i).timestamp;
        }
    }
//    std::cout << "Qd_summed trace = " << Qd_summed.trace() << std::endl;
    /// Last angular velocity (used for cloning when estimating time offset)
    Eigen::Matrix<double,3,1> last_w = Eigen::Matrix<double,3,1>::Zero();
    if(prop_data.size() > 1)
        last_w = prop_data.at(prop_data.size()-2).wm - state->_imu->bias_g();
    else if(!prop_data.empty())
        last_w = prop_data.at(prop_data.size()-1).wm - state->_imu->bias_g();

    /// Do the update to the covariance with our "summed" state transition and IMU noise addition...
    std::vector<Type*> Phi_order;
    Phi_order.push_back(state->_imu);
    StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_summed, Qd_summed);
    /// Set timestamp data
    state->_timestamp = timestamp;
    last_prop_time_offset = t_off_new;

    /// Perform stochastic cloning
    StateHelper::augment_clone(state, last_w);
    printf(BOLDGREEN "[Propagation] successfully propagated\n" RESET);
}

void Propagator::predict_and_compute(State *state, /// Pointer to state
                                     const IMUDATA data_minus, /// IMU readings at the beginning of interval
                                     const IMUDATA data_plus, /// IMU readings at the end of the interval
                                     Eigen::Matrix<double, 15, 15> &F, /// State-transition matrix over the interval
                                     Eigen::Matrix<double, 15, 15> &Qd) { /// Qd Discrete-time noise covariance over the interval
    // Set them to zero
    F.setZero();
    Qd.setZero();

    // Time elapsed over interval
    double dt = data_plus.timestamp-data_minus.timestamp;
    //assert(data_plus.timestamp>data_minus.timestamp);

    // Corrected imu measurements
    Eigen::Matrix<double,3,1> w_hat = data_minus.wm - state->_imu->bias_g();
    Eigen::Matrix<double,3,1> a_hat = data_minus.am - state->_imu->bias_a();
    Eigen::Matrix<double,3,1> w_hat2 = data_plus.wm - state->_imu->bias_g();
    Eigen::Matrix<double,3,1> a_hat2 = data_plus.am - state->_imu->bias_a();

    // Compute the new state mean value
    Eigen::Vector4d new_q;
    Eigen::Vector3d new_v, new_p;
    if(state->_options.use_rk4_integration) predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);
    else predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

    // Get the locations of each entry of the imu state
    int th_id = state->_imu->q()->id()-state->_imu->id();
    int p_id = state->_imu->p()->id()-state->_imu->id();
    int v_id = state->_imu->v()->id()-state->_imu->id();
    int bg_id = state->_imu->bg()->id()-state->_imu->id();
    int ba_id = state->_imu->ba()->id()-state->_imu->id();

    // Allocate noise Jacobian
    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();

    // Now compute Jacobian of new state wrt old state and noise
    if (state->_options.do_fej) {

        // This is the change in the orientation from the end of the last prop to the current prop
        // This is needed since we need to include the "k-th" updated orientation information
        Eigen::Matrix<double,3,3> Rfej = state->_imu->Rot_fe();
        Eigen::Matrix<double,3,3> dR = quat_2_Rot(new_q)*Rfej.transpose();

        Eigen::Matrix<double,3,1> v_fej = state->_imu->vel_fe();
        Eigen::Matrix<double,3,1> p_fej = state->_imu->pos_fe();

        F.block(th_id, th_id, 3, 3) = dR;
        F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
        //F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v-v_fej+_gravity*dt)*Rfej.transpose();
        //F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() * skew_x(Rfej*(new_v-v_fej+_gravity*dt));
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt)*Rfej.transpose();
        //F.block(p_id, th_id, 3, 3).noalias() = -0.5 * Rfej.transpose() * skew_x(2*Rfej*(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt));
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
        //G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-log_so3(dR)) * dt;
        G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
        G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
        G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
        G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();

    } else {

        Eigen::Matrix<double,3,3> R_Gtoi = state->_imu->Rot();

        F.block(th_id, th_id, 3, 3) = Exp_so3(-w_hat * dt);
        F.block(th_id, bg_id, 3, 3).noalias() = -Exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -Exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
        G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
        G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
    }

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
    double sigma_w_2 = _noises.sigma_w * _noises.sigma_w;
    double sigma_a_2 = _noises.sigma_a * _noises.sigma_a;
    double sigma_wb_2 = _noises.sigma_wb * _noises.sigma_wb;
    double sigma_ab_2 = _noises.sigma_ab * _noises.sigma_ab;
    Qc.block(0,0,3,3) = sigma_w_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(3,3,3,3) = sigma_a_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(6,6,3,3) = sigma_wb_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(9,9,3,3) = sigma_ab_2/dt*Eigen::Matrix<double,3,3>::Identity();

    // Compute the noise injected into the state over the interval
    Qd = G*Qc*G.transpose();
    Qd = 0.5*(Qd+Qd.transpose());

    //Now replace imu estimate and fej with propagated values
    Eigen::Matrix<double,16,1> imu_x = state->_imu->value();
    imu_x.block(0,0,4,1) = new_q;
    imu_x.block(4,0,3,1) = new_p;
    imu_x.block(7,0,3,1) = new_v;
    state->_imu->set_value(imu_x);
    state->_imu->set_fe(imu_x);
}

void Propagator::predict_mean_discrete(State *state, /// Pointer to state
                           double dt, /// Time we should integrate over
                           const Eigen::Vector3d &w_hat1, /// Angular velocity with bias removed
                           const Eigen::Vector3d &a_hat1, /// Linear acceleration with bias removed
                           const Eigen::Vector3d &w_hat2, /// Next angular velocity with bias removed
                           const Eigen::Vector3d &a_hat2, /// Next linear acceleration with bias removed
                           Eigen::Vector4d &new_q, /// The resulting new orientation after integration
                           Eigen::Vector3d &new_v, /// The resulting new velocity after integration
                           Eigen::Vector3d &new_p) /// The resulting new position after integration
{
    // If we are averaging the IMU, then do so
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    if (state->_options.imu_avg) {
        w_hat = .5*(w_hat1+w_hat2);
        a_hat = .5*(a_hat1+a_hat2);
    }

    // Pre-compute things
    double w_norm = w_hat.norm();
    Eigen::Matrix<double,4,4> I_4x4 = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,3,3> R_Gtoi = state->_imu->Rot();

    // Orientation: Equation (101) and (103) and of Trawny indirect TR
    Eigen::Matrix<double,4,4> bigO;
    if(w_norm > 1e-20) {
        bigO = cos(0.5*w_norm*dt)*I_4x4 + 1/w_norm*sin(0.5*w_norm*dt)*Omega(w_hat);
    } else {
        bigO = I_4x4 + 0.5*dt*Omega(w_hat);
    }
    new_q = quatnorm(bigO*state->_imu->quat());
//    new_q = rot_2_quat(Exp_so3(-w_hat*dt)*R_Gtoi);

    // Velocity: just the acceleration in the local frame, minus global gravity
    new_v = state->_imu->vel() + R_Gtoi.transpose()*a_hat*dt - _gravity*dt;

    // Position: just velocity times dt, with the acceleration integrated twice
    new_p = state->_imu->pos() + state->_imu->vel()*dt + 0.5*R_Gtoi.transpose()*a_hat*dt*dt - 0.5*_gravity*dt*dt;
}

void Propagator::predict_mean_rk4(State *state, /// Pointer to state
                      double dt, /// Time we should integrate over
                      const Eigen::Vector3d &w_hat1, /// Angular velocity with bias removed
                      const Eigen::Vector3d &a_hat1, /// Linear acceleration with bias removed
                      const Eigen::Vector3d &w_hat2, /// Next angular velocity with bias removed
                      const Eigen::Vector3d &a_hat2, /// Next linear acceleration with bias removed
                      Eigen::Vector4d &new_q, /// The resulting new orientation after integration
                      Eigen::Vector3d &new_v, /// The resulting new velocity after integration
                      Eigen::Vector3d &new_p)/// The resulting new position after integration
{
    /// Pre-compute things
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    Eigen::Vector3d w_alpha = (w_hat2-w_hat1)/dt;
    Eigen::Vector3d a_jerk = (a_hat2-a_hat1)/dt;

    /// y0 ================
    Eigen::Vector4d q_0 = state->_imu->quat();
    Eigen::Vector3d p_0 = state->_imu->pos();
    Eigen::Vector3d v_0 = state->_imu->vel();

    /// k1 ================
    Eigen::Vector4d dq_0 = {0,0,0,1};
    Eigen::Vector4d q0_dot = 0.5*Omega(w_hat)*dq_0;
    Eigen::Vector3d p0_dot = v_0;
    Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0,q_0));
    Eigen::Vector3d v0_dot = R_Gto0.transpose()*a_hat-_gravity;

    Eigen::Vector4d k1_q = q0_dot*dt;
    Eigen::Vector3d k1_p = p0_dot*dt;
    Eigen::Vector3d k1_v = v0_dot*dt;

    /// k2 ================
    w_hat += 0.5*w_alpha*dt;
    a_hat += 0.5*a_jerk*dt;

    Eigen::Vector4d dq_1 = quatnorm(dq_0+0.5*k1_q);
    //Eigen::Vector3d p_1 = p_0+0.5*k1_p;
    Eigen::Vector3d v_1 = v_0+0.5*k1_v;

    Eigen::Vector4d q1_dot = 0.5*Omega(w_hat)*dq_1;
    Eigen::Vector3d p1_dot = v_1;
    Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1,q_0));
    Eigen::Vector3d v1_dot = R_Gto1.transpose()*a_hat-_gravity;

    Eigen::Vector4d k2_q = q1_dot*dt;
    Eigen::Vector3d k2_p = p1_dot*dt;
    Eigen::Vector3d k2_v = v1_dot*dt;

    /// k3 ================
    Eigen::Vector4d dq_2 = quatnorm(dq_0+0.5*k2_q);
    //Eigen::Vector3d p_2 = p_0+0.5*k2_p;
    Eigen::Vector3d v_2 = v_0+0.5*k2_v;

    Eigen::Vector4d q2_dot = 0.5*Omega(w_hat)*dq_2;
    Eigen::Vector3d p2_dot = v_2;
    Eigen::Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2,q_0));
    Eigen::Vector3d v2_dot = R_Gto2.transpose()*a_hat-_gravity;

    Eigen::Vector4d k3_q = q2_dot*dt;
    Eigen::Vector3d k3_p = p2_dot*dt;
    Eigen::Vector3d k3_v = v2_dot*dt;

    /// k4 ================
    w_hat += 0.5*w_alpha*dt;
    a_hat += 0.5*a_jerk*dt;

    Eigen::Vector4d dq_3 = quatnorm(dq_0+k3_q);
    //Eigen::Vector3d p_3 = p_0+k3_p;
    Eigen::Vector3d v_3 = v_0+k3_v;

    Eigen::Vector4d q3_dot = 0.5*Omega(w_hat)*dq_3;
    Eigen::Vector3d p3_dot = v_3;
    Eigen::Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3,q_0));
    Eigen::Vector3d v3_dot = R_Gto3.transpose()*a_hat-_gravity;

    Eigen::Vector4d k4_q = q3_dot*dt;
    Eigen::Vector3d k4_p = p3_dot*dt;
    Eigen::Vector3d k4_v = v3_dot*dt;

    /// y+dt ================
    Eigen::Vector4d dq = quatnorm(dq_0+(1.0/6.0)*k1_q+(1.0/3.0)*k2_q+(1.0/3.0)*k3_q+(1.0/6.0)*k4_q);
    new_q = quat_multiply(dq, q_0);
    new_p = p_0+(1.0/6.0)*k1_p+(1.0/3.0)*k2_p+(1.0/3.0)*k3_p+(1.0/6.0)*k4_p;
    new_v = v_0+(1.0/6.0)*k1_v+(1.0/3.0)*k2_v+(1.0/3.0)*k3_v+(1.0/6.0)*k4_v;
}