//
// Created by usl on 11/6/20.
//

#ifndef LINCALIB_IMU_H
#define LINCALIB_IMU_H

#include "Pose.h"
#include "utils/quat_ops.h"

namespace lin_type {


    /**
     * @brief Derived Type class that implements an IMU state
     *
     * Contains a Pose, Vec velocity, Vec gyro bias, and Vec accel bias.
     * This should be similar to that of the standard MSCKF state besides the ordering.
     * The pose is first, followed by velocity, etc.
     */
    class IMU : public Type {

    public:

        IMU() : Type(15) {
            _pose = new Pose();
            _v = new Vec(3);
            _bg = new Vec(3);
            _ba = new Vec(3);

            Eigen::Matrix<double, 16, 1> imu0;
            imu0.setZero();
            imu0(3) = 1.0;

            set_value(imu0);
            set_fe(imu0);
        }

        ~IMU() {
            delete _pose;
            delete _v;
            delete _bg;
            delete _ba;
        }

        /**
         * @brief Sets id used to track location of variable in the filter covariance
         *
         * Note that we update the sub-variables also.
         *
         * @param new_id entry in filter covariance corresponding to this variable
         */
        void set_local_id(int new_id) override {
            _id = new_id;
            _pose->set_local_id(new_id);
            _v->set_local_id(_pose->id()+_pose->size());
            _bg->set_local_id(_v->id()+_v->size());
            _ba->set_local_id(_bg->id()+_bg->size());
        }

        /**
        * @brief Performs update operation using JPLQuat update for orientation, then vector updates for
        * position, velocity, gyro bias, and accel bias (in that order).
         *
        * @param dx 15 DOF vector encoding update using the following order (q, p, v, bg, ba)
        */
        void update(const Eigen::VectorXd dx) override {

            assert(dx.rows() == _size);

            Eigen::Matrix<double, 16, 1> newX = _value;

            Eigen::Matrix<double, 4, 1> dq;
            dq << .5 * dx.block(0, 0, 3, 1), 1.0;
            dq = lin_core::quatnorm(dq);

            newX.block(0, 0, 4, 1) = lin_core::quat_multiply(dq, quat());
            newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

            newX.block(7, 0, 3, 1) += dx.block(6, 0, 3, 1);
            newX.block(10, 0, 3, 1) += dx.block(9, 0, 3, 1);
            newX.block(13, 0, 3, 1) += dx.block(12, 0, 3, 1);

            set_value(newX);

        }


        /**
         * @brief Sets the value of the estimate
         * @param new_value New value we should set
         */
        void set_value(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 16);
            assert(new_value.cols() == 1);

            _pose->set_value(new_value.block(0, 0, 7, 1));
            _v->set_value(new_value.block(7, 0, 3, 1));
            _bg->set_value(new_value.block(10, 0, 3, 1));
            _ba->set_value(new_value.block(13, 0, 3, 1));

            _value = new_value;
        }

        /**
         * @brief Sets the value of the first estimate
         * @param new_value New value we should set
         */
        void set_fe(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 16);
            assert(new_value.cols() == 1);

            _pose->set_fe(new_value.block(0, 0, 7, 1));
            _v->set_fe(new_value.block(7, 0, 3, 1));
            _bg->set_fe(new_value.block(10, 0, 3, 1));
            _ba->set_fe(new_value.block(13, 0, 3, 1));

            _fe = new_value;
        }

        Type *clone() override {
            Type *Clone = new IMU();
            Clone->set_value(value());
            Clone->set_fe(fe());
            return Clone;
        }

        /**
         * @brief Used to find the components inside the IMU if needed.
         * If the passed variable is a sub-variable or the current variable this will return it.
         * Otherwise it will return a nullptr, meaning that it was unable to be found.
         *
         * @param check variable to find
         */
        Type *check_if_same_variable(const Type *check) override {
            if (check == this) {
                return this;
            } else if (check == _pose) {
                return _pose;
            } else if (check == q()) {
                return q();
            } else if (check == p()) {
                return p();
            } else if (check == _v) {
                return _v;
            } else if (check == _bg) {
                return bg();
            } else if (check == _ba) {
                return ba();
            } else {
                return nullptr;
            }
        }

        /// Rotation access
        Eigen::Matrix<double, 3, 3> Rot() const {
            return _pose->Rot();
        }

        /// FEJ Rotation access
        Eigen::Matrix<double, 3, 3> Rot_fe() const {
            return _pose->Rot_fe();
        }

        /// Rotation access quaternion
        Eigen::Matrix<double, 4, 1> quat() const {
            return _pose->quat();
        }

        /// FEJ Rotation access quaternion
        Eigen::Matrix<double, 4, 1> quat_fe() const {
            return _pose->quat_fe();
        }


        /// Position access
        Eigen::Matrix<double, 3, 1> pos() const {
            return _pose->pos();
        }

        /// FEJ position access
        Eigen::Matrix<double, 3, 1> pos_fe() const {
            return _pose->pos_fe();
        }

        /// Velocity access
        Eigen::Matrix<double, 3, 1> vel() const {
            return _v->value();
        }

        // FEJ velocity access
        Eigen::Matrix<double, 3, 1> vel_fe() const {
            return _v->fe();
        }

        /// Gyro bias access
        Eigen::Matrix<double, 3, 1> bias_g() const {
            return _bg->value();
        }

        /// FEJ gyro bias access
        Eigen::Matrix<double, 3, 1> bias_g_fe() const {
            return _bg->fe();
        }

        /// Accel bias access
        Eigen::Matrix<double, 3, 1> bias_a() const {
            return _ba->value();
        }

        // FEJ accel bias access
        Eigen::Matrix<double, 3, 1> bias_a_fe() const {
            return _ba->fe();
        }

        /// Pose type access
        Pose *pose() {
            return _pose;
        }

        /// Quaternion type access
        Quat *q() {
            return _pose->q();
        }

        /// Position type access
        Vec *p() {
            return _pose->p();
        }

        /// Velocity type access
        Vec *v() {
            return _v;
        }

        /// Gyroscope bias access
        Vec *bg() {
            return _bg;
        }

        /// Acceleration bias access
        Vec *ba() {
            return _ba;
        }

    protected:

        /// Pose subvariable
        Pose *_pose;

        /// Velocity subvariable
        Vec *_v;

        /// Gyroscope bias subvariable
        Vec *_bg;
        
        /// Acceleration bias subvariable
        Vec *_ba;
    };

};
#endif //LINCALIB_IMU_H
