//
// Created by usl on 4/23/21.
//

#ifndef LINKALIBR_POINTTOPLANEFACTOR2_H
#define LINKALIBR_POINTTOPLANEFACTOR2_H
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <ostream>

#include "PreIntegratedIMUMeasurements.h"

using namespace gtsam;
namespace lin_estimator {
    /**
    * A class for point to plane constraint
    */
    class PointToPlaneFactor2 : public NoiseModelFactor1<Pose3> {
    private:
        typedef PointToPlaneFactor2 This;
        typedef NoiseModelFactor1<Pose3> Base;

        PreIntegratedIMUMeasurements preintegrated_imu_measurements_;
        Vector4 plane_param_measurement_;
        Point3 lidar_point_measurement_;
        Pose3 current_pose_;
        Vector3 current_velocity_;

    public:
        /// Shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<PointToPlaneFactor2> shared_ptr;

        /** default constructor - only use for serialization */
        PointToPlaneFactor2(Key key1,
                            const PreIntegratedIMUMeasurements& preintegrated_imu_measurements,
                            const Vector4& plane_param_measurement,
                            const Point3& lidar_point_measurement,
                            const Pose3& curr_pose,
                            const Vector3 curr_velocity,
                            const SharedNoiseModel& model)
                : Base(model, key1),
                  preintegrated_imu_measurements_(preintegrated_imu_measurements),
                  plane_param_measurement_(plane_param_measurement),
                  lidar_point_measurement_(lidar_point_measurement),
                  current_pose_(curr_pose),
                  current_velocity_(curr_velocity)
        {}

        ~PointToPlaneFactor2(){}

        /** Implement functions needed for Testable */

        /** print **/
        virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            std::cout << s << "PointToPlaneFactor("
                      << keyFormatter(this->key()) <<")" << std::endl;
            std::cout << "PreintegratedImuMeasurements\n"
                      << "deltaRij: \n" << preintegrated_imu_measurements_.deltaR.matrix() << "\n"
                      << "deltaVij: \t" << preintegrated_imu_measurements_.deltaV.transpose() << "\n"
                      << "deltaPij: \t" << preintegrated_imu_measurements_.deltaP.transpose() << "\n"
                      << "deltaTij: \t" << preintegrated_imu_measurements_.deltaT << "\n"
                      << "Gravity Vector: \t" << preintegrated_imu_measurements_.gravity.transpose() << "\n"
                      << "H_bias_omega: \n" << preintegrated_imu_measurements_.H_bias_omega << "\n"
                      << "H_bias_accel: \n" << preintegrated_imu_measurements_.H_bias_accel << "\n"
                      << "plane_param: \t" << plane_param_measurement_.transpose() << "\n"
                      << "lidar_point: \t" << lidar_point_measurement_.transpose() << std::endl;
            this->noiseModel_->print("  noise model: ");
        }

        /** equals **/

        virtual bool equals(const NonlinearFactor& expected,
                            double tol = 1e-9) const {
            const This* e = dynamic_cast<const This*>(&expected);
            const bool base = Base::equals(*e, tol); // equating base structures
            const bool pim_deltaR = traits<Rot3>::Equals(this->preintegrated_imu_measurements_.deltaR, e->preintegrated_imu_measurements_.deltaR);
            const bool pim_deltaV = traits<Vector3>::Equals(this->preintegrated_imu_measurements_.deltaV, e->preintegrated_imu_measurements_.deltaV);
            const bool pim_deltaP = traits<Vector3>::Equals(this->preintegrated_imu_measurements_.deltaP, e->preintegrated_imu_measurements_.deltaP);
            const bool pim_deltaT = traits<double>::Equals(this->preintegrated_imu_measurements_.deltaT, e->preintegrated_imu_measurements_.deltaT);
            const bool pim_gravity = traits<Vector3>::Equals(this->preintegrated_imu_measurements_.gravity, e->preintegrated_imu_measurements_.gravity);
            const bool pim_H_bias_omega = traits<Matrix93>::Equals(this->preintegrated_imu_measurements_.H_bias_omega, e->preintegrated_imu_measurements_.H_bias_omega);
            const bool pim_H_bias_accel = traits<Matrix93>::Equals(this->preintegrated_imu_measurements_.H_bias_accel, e->preintegrated_imu_measurements_.H_bias_accel);
            const bool pp = traits<Vector4>::Equals(this->plane_param_measurement_, e->plane_param_measurement_, tol);
            const bool lp = traits<Vector3>::Equals(this->lidar_point_measurement_, e->lidar_point_measurement_, tol);
            const bool bool_pose = traits<Pose3>::Equals(this->current_pose_, e->current_pose_, tol);
            const bool bool_velocity = traits<Vector3>::Equals(this->current_velocity_, e->current_velocity_, tol);
            return e != nullptr && base && pim_deltaR && pim_deltaV
                   && pim_deltaP && pim_deltaT && pim_gravity
                   && pim_H_bias_omega && pim_H_bias_accel
                   && pp && lp && bool_pose && bool_velocity;
        }

        /** implement functions needed to derive from Factor */

        /// Vector of errors
        Vector evaluateError(const Pose3& Tc,
                             boost::optional<Matrix&> H1 = boost::none) const override;

        /** return measured **/
        const PreIntegratedIMUMeasurements& preintegrated_imu_measurements() const {return preintegrated_imu_measurements_;};
        const Vector4& plane_param_measurement() const {return plane_param_measurement_;}
        const Point3& lidar_point_measurement() const {return lidar_point_measurement_;}
        /** Residual/Error and Jacobian Calculator, Jacobians determined using GTSAM functions **/
        Vector1 computeErrorAndJacobians(const Pose3& Tc,
                                         OptionalJacobian<1, 6> H) const;
    private:
        friend class boost::serialization::access;
        template <class ARCHIVE>
        void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
            ar & boost::serialization::make_nvp(
                    "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(preintegrated_imu_measurements_);
            ar & BOOST_SERIALIZATION_NVP(plane_param_measurement_);
            ar & BOOST_SERIALIZATION_NVP(lidar_point_measurement_);
        }
    };
}
#endif //LINKALIBR_POINTTOPLANEFACTOR2_H
