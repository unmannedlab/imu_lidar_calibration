//
// Created by usl on 11/5/20.
//

#ifndef LINCALIB_TYPE_H
#define LINCALIB_TYPE_H

#include <Eigen/Dense>

namespace lin_type {


    /**
     * @brief Base class for estimated variables.
     *
     * This class is used how variables are represented or updated (e.g., vectors or quaternions).
     * Each variable is defined by its error state size and its location in the covariance matrix.
     * We additionally require all sub-types to have a update procedure.
     */
    class Type {

    public:

        /**
        * @brief Default constructor for our Type
         *
        * @param size_ degrees of freedom of variable (i.e., the size of the error state)
        */
        Type(int size_) {
            _size = size_;
        }

        virtual ~Type() {};

        /**
        * @brief Sets id used to track location of variable in the filter covariance
         *
        * @param new_id entry in filter covariance corresponding to this variable
        */
        virtual void set_local_id(int new_id) {
            _id = new_id;
        }

        /**
        * @brief Access to variable id (i.e. its location in the covariance)
        */
        int id() {
            return _id;
        }

        /**
        * @brief Access to variable size (i.e. its error state size)
        */
        int size() {
            return _size;
        }

        /**
         * @brief Update variable due to perturbation of error state
         *
         * @param dx Perturbation used to update the variable through a defined "boxplus" operation
         */
        virtual void update(const Eigen::VectorXd dx) = 0;

        /**
         * @brief Access variable's estimate
         */
        virtual Eigen::MatrixXd value() const {
            return _value;
        }

        /**
         * @brief Access variable's first-estimate
         */
        virtual Eigen::MatrixXd fe() const {
            return _fe;
        }

        /**
         * @brief Overwrite value of state's estimate
         * @param new_value New value that will overwrite state's value
         */
        virtual void set_value(const Eigen::MatrixXd new_value) {
            assert(_value.rows()==new_value.rows());
            assert(_value.cols()==new_value.cols());
            _value = new_value;
        }

        /**
         * @brief Overwrite value of first-estimate
         * @param new_value New value that will overwrite state's fej
         */
        virtual void set_fe(const Eigen::MatrixXd new_value) {
            assert(_fe.rows()==new_value.rows());
            assert(_fe.cols()==new_value.cols());
            _fe = new_value;
        }

        /**
        * @brief Create a clone of this variable
        */
        virtual Type *clone() = 0;

        /**
         * @brief Determine if "check" is the same variable
         * If the passed variable is a sub-variable or the current variable this will return it.
         * Otherwise it will return a nullptr, meaning that it was unable to be found.
         *
         * @param check Type pointer to compare to
         */
        virtual Type *check_if_same_variable(const Type *check) {
            if (check == this) {
                return this;
            } else {
                return nullptr;
            }
        }

    protected:

        /// First-estimate
        Eigen::MatrixXd _fe;

        /// Current best estimate
        Eigen::MatrixXd _value;

        /// Location of error state in covariance
        int _id = -1;

        /// Dimension of error state
        int _size = -1;


    };
}
#endif //LINCALIB_TYPE_H
