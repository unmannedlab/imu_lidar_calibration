//
// Created by usl on 11/5/20.
//

#ifndef LINCALIB_QUAT_H
#define LINCALIB_QUAT_H

#include "Type.h"
#include "utils/quat_ops.h"

namespace lin_type {

    /**
     * @brief Derived Type class that implements JPL quaternion
     *
     * This quaternion uses a left-multiplicative error state and follows the "JPL convention".
     * Please checkout our utility functions in the quat_ops.h file.
     * We recommend that people new quaternions check out the following resources:
     * - http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
     * - ftp://naif.jpl.nasa.gov/pub/naif/misc/Quaternion_White_Paper/Quaternions_White_Paper.pdf
     */
    class Quat : public Type {

    public:

        Quat() : Type(3) {
            Eigen::Matrix<double, 4, 1> q0;
            q0.setZero();
            q0(3) = 1.0;
            set_value(q0);
            set_fe(q0);
        }

        ~Quat() {}

        /**
         * @brief Implements update operation by left-multiplying the current
         * quaternion with a quaternion built from a small axis-angle perturbation.
         *
         * @f[
         * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
         * @f]
         *
         * @param dx Axis-angle representation of the perturbing quaternion
         */
        void update(const Eigen::VectorXd dx) override {

            assert(dx.rows() == _size);

            //Build perturbing quaternion
            Eigen::Matrix<double, 4, 1> dq;
            dq << .5 * dx, 1.0;
            dq = lin_core::quatnorm(dq);

            //Update estimate and recompute R
            set_value(lin_core::quat_multiply(dq, _value));

        }

        /**
        * @brief Sets the value of the estimate and recomputes the internal rotation matrix
        * @param new_value New value for the quaternion estimate
        */
        void set_value(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 4);
            assert(new_value.cols() == 1);

            _value = new_value;

            //compute associated rotation
            _R = lin_core::quat_2_Rot(new_value);
        }

        Type *clone() override {
            Type *Clone = new Quat();
            Clone->set_value(value());
            Clone->set_fe(fe());
            return Clone;
        }

        /**
        * @brief Sets the fej value and recomputes the fej rotation matrix
        * @param new_value New value for the quaternion estimate
        */
        void set_fe(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 4);
            assert(new_value.cols() == 1);

            _fe = new_value;

            //compute associated rotation
            _Rfe = lin_core::quat_2_Rot(new_value);
        }

        /// Rotation access
        Eigen::Matrix<double, 3, 3> Rot() const {
            return _R;
        }

        /// FEJ Rotation access
        Eigen::Matrix<double, 3, 3> Rot_fe() const {
            return _Rfe;
        }

    protected:

        // Stores the rotation
        Eigen::Matrix<double, 3, 3> _R;

        // Stores the first-estimate rotation
        Eigen::Matrix<double, 3, 3> _Rfe;

    };
};

#endif //LINCALIB_QUAT_H
