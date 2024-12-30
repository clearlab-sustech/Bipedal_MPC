#pragma once

#include <core/types.h>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/math/rpy.hpp>

namespace clear
{
        matrix3_t getJacobiFromOmegaToRPY(const vector3_t &rpy);

        matrix3_t getJacobiFromRPYToOmega(const vector3_t &rpy);

        matrix3_t getJacobiDotFromRPYToOmega(const vector3_t &rpy,
                                             const vector3_t &rpy_dot);

        vector3_t toEulerAngles(const Eigen::Quaternion<scalar_t> &q);

        vector3_t toEulerAngles(const matrix3_t &R);

        Eigen::Quaternion<scalar_t> toQuaternion(const matrix3_t &R);

        Eigen::Quaternion<scalar_t>
        toQuaternion(const vector3_t &rpy); // roll (x), pitch (Y), yaw (z)

        matrix3_t
        toRotationMatrix(const vector3_t &rpy); // roll (x), pitch (Y), yaw (z)

        matrix3_t skew(const vector3_t &vec);

        vector3_t computeEulerAngleErr(const vector3_t &rpy_m,
                                       const vector3_t &rpy_d);
} // namespace clear
