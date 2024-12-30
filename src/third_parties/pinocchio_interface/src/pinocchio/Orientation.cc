#include "pinocchio/Orientation.h"
#include <iostream>

namespace clear
{
  matrix3_t getJacobiFromOmegaToRPY(const vector3_t &rpy)
  {
    return pinocchio::rpy::computeRpyJacobianInverse(
        rpy, pinocchio::LOCAL_WORLD_ALIGNED);
  }

  matrix3_t getJacobiFromRPYToOmega(const vector3_t &rpy)
  {
    return pinocchio::rpy::computeRpyJacobian(rpy,
                                              pinocchio::LOCAL_WORLD_ALIGNED);
  }

  matrix3_t getJacobiDotFromRPYToOmega(const vector3_t &rpy,
                                       const vector3_t &rpy_dot)
  {
    return pinocchio::rpy::computeRpyJacobianTimeDerivative(
        rpy, rpy_dot, pinocchio::LOCAL_WORLD_ALIGNED);
  }

  Eigen::Quaternion<scalar_t> toQuaternion(const matrix3_t &R)
  {
    Eigen::Quaternion<scalar_t> q;
    pinocchio::quaternion::assignQuaternion(q, R);
    return q;
  }

  vector3_t toEulerAngles(const matrix3_t &R)
  {
    return pinocchio::rpy::matrixToRpy(R);
  }

  vector3_t toEulerAngles(const Eigen::Quaternion<scalar_t> &q)
  {
    return pinocchio::rpy::matrixToRpy(q.toRotationMatrix());
  }

  Eigen::Quaternion<scalar_t>
  toQuaternion(const vector3_t &rpy) // roll (x), pitch (Y), yaw (z)
  {
    return toQuaternion(toRotationMatrix(rpy));
  }

  matrix3_t toRotationMatrix(const vector3_t &rpy) // roll (x), pitch (Y), yaw (z)
  {
    return pinocchio::rpy::rpyToMatrix(rpy);
  }

  matrix3_t skew(const vector3_t &vec)
  {
    return (matrix3_t() << 0, -vec.z(), vec.y(), vec.z(), 0, -vec.x(), -vec.y(),
            vec.x(), 0)
        .finished();
  }

  vector3_t computeEulerAngleErr(const vector3_t &rpy_m,
                                 const vector3_t &rpy_d)
  {
    vector3_t rpy_err = rpy_m - rpy_d;
    while (rpy_err.norm() > 1.5 * M_PI)
    {
      if (abs(rpy_err(0)) > M_PI)
      {
        rpy_err(0) += (rpy_err(0) > 0 ? -2.0 : 2.0) * M_PI;
      }
      if (abs(rpy_err(1)) > M_PI)
      {
        rpy_err(1) += (rpy_err(1) > 0 ? -2.0 : 2.0) * M_PI;
      }
      if (abs(rpy_err(2)) > M_PI)
      {
        rpy_err(2) += (rpy_err(2) > 0 ? -2.0 : 2.0) * M_PI;
      }
    }
    return rpy_err;
  }

} // namespace clear
