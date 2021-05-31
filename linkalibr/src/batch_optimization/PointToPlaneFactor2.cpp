//
// Created by usl on 4/23/21.
//

#include "PointToPlaneFactor2.h"

namespace lin_estimator {

    Vector1 PointToPlaneFactor2::computeErrorAndJacobians(const Pose3& Tc, OptionalJacobian<1, 6> H1) const {
        /// Pre-integrated measurements
        Rot3 deltaR = preintegrated_imu_measurements_.deltaR;
        Vector3 deltaP = preintegrated_imu_measurements_.deltaP;
        Vector3 deltaV = preintegrated_imu_measurements_.deltaV;
        double deltaT = preintegrated_imu_measurements_.deltaT;
        Vector3 gravity = preintegrated_imu_measurements_.gravity;

        Pose3 TA = Pose3(Rot3::identity(), current_velocity_*deltaT + 0.5*gravity*deltaT*deltaT);
        Pose3 TB = Pose3(deltaR, deltaP);
        Pose3 I1_T_Ik = Pose3();

        Matrix H_1, H_2, H_3, H_4, H_5, H_6, H_7, H_8, H_9;
        Pose3 L1_T_Lkplus = Tc.inverse(H_1).compose(TA, H_2, H_3).compose(I1_T_Ik, H_4, H_5).compose(TB, H_6, H_7).compose(Tc, H_8, H_9);

        /// Jacobian of L1_T_Lmplusi wrt Tc
        Matrix6 H_L1_T_Lkplus_Tc;
        H_L1_T_Lkplus_Tc = H_9 + H_8*H_6*H_4*H_2*H_1;

        /// Transform lidar point measurement
        Matrix36 H_xL1_L1TLkplus; /// Jacobian of x_L1 wrt L1_T_Lmplusi (3 x 6)
        Point3 x_L1 = L1_T_Lkplus.transformFrom(lidar_point_measurement_, H_xL1_L1TLkplus);

        /// Point to plane constraint
        Point3 n_L1 = Point3(plane_param_measurement_.x(), plane_param_measurement_.y(), plane_param_measurement_.z());
        double d_L1 = plane_param_measurement_.z();

        /// Residual
        Vector1 res = Vector1((n_L1.transpose()*x_L1 + d_L1));

        /// Jacobian of res wrt xL1 ( 1 x 3 )
        Matrix H_res_xL1 = n_L1.transpose();

        if (H1)
            (*H1) = (Matrix16() << H_res_xL1*H_xL1_L1TLkplus*H_L1_T_Lkplus_Tc).finished();
        return res;
    }

    Vector PointToPlaneFactor2::evaluateError(const Pose3 &Tc, boost::optional<Matrix &> H1) const {
        Vector1 error = computeErrorAndJacobians(Tc, H1);
        return error;
    }
}