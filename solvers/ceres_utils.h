/*
 * Copyright 2018 Simbe Robotics
 * Author: Steve Macenski
 */

#ifndef SOLVERS__CERES_UTILS_H_
#define SOLVERS__CERES_UTILS_H_

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <utility>

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline std::size_t GetHash(const int & x, const int & y)
{
  return (std::hash<double>()(x) ^ (std::hash<double>()(y) << 1)) >> 1;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Normalizes the angle in radians between [-pi and pi).
template<typename T>
inline T NormalizeAngle(const T & angle_radians)
{
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class AngleLocalParameterization
{
public:
  template<typename T>
  bool operator()(
    const T * theta_radians, const T * delta_theta_radians,
    T * theta_radians_plus_delta) const
  {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization * Create()
  {
    return new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>;
  }
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template<typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
{
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);
  Eigen::Matrix<T, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class PoseGraph2dErrorTerm
{
public:
  PoseGraph2dErrorTerm(
    double x_ab, double y_ab, double yaw_ab_radians,
    const Eigen::Matrix3d & sqrt_information)
  : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians), sqrt_information_(sqrt_information)
  {
  }

  template<typename T>
  bool operator()(
    const T * const x_a, const T * const y_a, const T * const yaw_a,
    const T * const x_b, const T * const y_b, const T * const yaw_b,
    T * residuals_ptr) const
  {
    const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
    const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);
    residuals_map.template head<2>() = RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) -
      p_ab_.cast<T>();
    residuals_map(2) = NormalizeAngle((*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));
    // Scale the residuals by the square root information
    // matrix to account for the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;
    return true;
  }

  static ceres::CostFunction * Create(
    double x_ab, double y_ab, double yaw_ab_radians,
    const Eigen::Matrix3d & sqrt_information)
  {
    return new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
      new PoseGraph2dErrorTerm(
        x_ab, y_ab, yaw_ab_radians,
        sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Same measurement model as PoseGraph2dErrorTerm, but for a scan-matcher measurement that was
// made in the *sensor* frame rather than the robot frame, with the lidar<->base_link extrinsic
// (x_e, y_e, yaw_e) as a live, jointly-optimized parameter shared across every edge from that
// sensor. (x_ab, y_ab, yaw_ab) is the frozen measured sensor-frame relative pose of scan B as
// seen from scan A (LinkInfo::GetSensorPoseDifference()).
//
// IMPORTANT: the rotation reference for both endpoints must be the full composed sensor
// heading (yaw_a + yaw_e / yaw_b + yaw_e), not the robot heading alone. Rotating by yaw_a alone
// makes yaw_e cancel out of every residual identically, leaving it permanently unobservable
// with zero gradient -- it will compile and run and simply never move off its prior. See the
// degenerate-motion regression test in test/ceres_extrinsic_test.cpp.
class SensorExtrinsicPoseGraph2dErrorTerm
{
public:
  SensorExtrinsicPoseGraph2dErrorTerm(
    double x_ab, double y_ab, double yaw_ab_radians,
    const Eigen::Matrix3d & sqrt_information)
  : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians), sqrt_information_(sqrt_information)
  {
  }

  template<typename T>
  bool operator()(
    const T * const x_a, const T * const y_a, const T * const yaw_a,
    const T * const x_b, const T * const y_b, const T * const yaw_b,
    const T * const x_e, const T * const y_e, const T * const yaw_e,
    T * residuals_ptr) const
  {
    const Eigen::Matrix<T, 2, 1> ext(*x_e, *y_e);

    const T sensor_yaw_a = *yaw_a + *yaw_e;
    const Eigen::Matrix<T, 2, 1> sensor_p_a =
      Eigen::Matrix<T, 2, 1>(*x_a, *y_a) + RotationMatrix2D(*yaw_a) * ext;

    const T sensor_yaw_b = *yaw_b + *yaw_e;
    const Eigen::Matrix<T, 2, 1> sensor_p_b =
      Eigen::Matrix<T, 2, 1>(*x_b, *y_b) + RotationMatrix2D(*yaw_b) * ext;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);
    residuals_map.template head<2>() =
      RotationMatrix2D(sensor_yaw_a).transpose() * (sensor_p_b - sensor_p_a) - p_ab_.cast<T>();
    residuals_map(2) = NormalizeAngle((sensor_yaw_b - sensor_yaw_a) - static_cast<T>(yaw_ab_radians_));
    // Scale the residuals by the square root information
    // matrix to account for the measurement uncertainty.
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;
    return true;
  }

  static ceres::CostFunction * Create(
    double x_ab, double y_ab, double yaw_ab_radians,
    const Eigen::Matrix3d & sqrt_information)
  {
    return new ceres::AutoDiffCostFunction<SensorExtrinsicPoseGraph2dErrorTerm,
             3, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
      new SensorExtrinsicPoseGraph2dErrorTerm(
        x_ab, y_ab, yaw_ab_radians,
        sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // The position of B relative to A in the A frame, in sensor-frame measurement space.
  const Eigen::Vector2d p_ab_;
  // The orientation of frame B relative to frame A, in sensor-frame measurement space.
  const double yaw_ab_radians_;
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

// Anchors a live sensor extrinsic estimate (x_e, y_e, yaw_e) to a fixed nominal value (usually
// the TF-derived CAD/mounting value read at startup) with a tight Gaussian prior, so the joint
// optimization can refine the extrinsic online without it drifting arbitrarily far from the
// known-good starting estimate.
class ExtrinsicPriorErrorTerm
{
public:
  ExtrinsicPriorErrorTerm(
    double x_nominal, double y_nominal, double yaw_nominal,
    double stddev_xy, double stddev_yaw)
  : x_nominal_(x_nominal), y_nominal_(y_nominal), yaw_nominal_(yaw_nominal),
    inv_stddev_xy_(1.0 / stddev_xy), inv_stddev_yaw_(1.0 / stddev_yaw)
  {
  }

  template<typename T>
  bool operator()(
    const T * const x_e, const T * const y_e, const T * const yaw_e,
    T * residuals_ptr) const
  {
    residuals_ptr[0] = (*x_e - static_cast<T>(x_nominal_)) * static_cast<T>(inv_stddev_xy_);
    residuals_ptr[1] = (*y_e - static_cast<T>(y_nominal_)) * static_cast<T>(inv_stddev_xy_);
    residuals_ptr[2] = NormalizeAngle(*yaw_e - static_cast<T>(yaw_nominal_)) *
      static_cast<T>(inv_stddev_yaw_);
    return true;
  }

  static ceres::CostFunction * Create(
    double x_nominal, double y_nominal, double yaw_nominal,
    double stddev_xy, double stddev_yaw)
  {
    return new ceres::AutoDiffCostFunction<ExtrinsicPriorErrorTerm, 3, 1, 1, 1>(
      new ExtrinsicPriorErrorTerm(x_nominal, y_nominal, yaw_nominal, stddev_xy, stddev_yaw));
  }

private:
  const double x_nominal_, y_nominal_, yaw_nominal_;
  const double inv_stddev_xy_, inv_stddev_yaw_;
};

#endif  // SOLVERS__CERES_UTILS_H_
