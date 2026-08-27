/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef SOLVERS__CERES_SOLVER_HPP_
#define SOLVERS__CERES_SOLVER_HPP_

#include <math.h>
#include <ceres/local_parameterization.h>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <cmath>
#include "karto_sdk/Mapper.h"
#include "solvers/ceres_utils.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/toolbox_types.hpp"

namespace solver_plugins
{

using namespace ::toolbox_types;  // NOLINT

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

public:
  // Get corrected poses after optimization
  virtual const karto::ScanSolver::IdPoseVector & GetCorrections() const;

  virtual void Compute();  // Solve
  virtual void Clear();  // Resets the corrections
  virtual void Reset();  // Resets the solver plugin clean
  virtual void Configure(rclcpp::Node::SharedPtr node);

  // Adds a node to the solver
  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex);
  // Adds a constraint to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge);
  // Get graph stored
  virtual std::unordered_map<int, Eigen::Vector3d> * getGraph();
  // Removes a node from the solver correction table
  virtual void RemoveNode(kt_int32s id);
  // Removes constraints from the optimization problem
  virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId);

  // change a node's pose
  virtual void ModifyNode(const int & unique_id, Eigen::Vector3d pose);
  // get a node's current pose yaw
  virtual void GetNodeOrientation(const int & unique_id, double & pose);

  // Get the live, jointly-optimized sensor extrinsic offset estimates
  virtual const std::unordered_map<std::string, karto::Pose2> & GetSensorOffsetCorrections()
  const;
  // Get the marginal covariance of each sensor extrinsic offset estimate
  virtual const std::unordered_map<std::string, Eigen::Matrix3d> & GetSensorOffsetCovariances()
  const;
  // Get the frozen nominal offset each sensor's extrinsic prior was anchored to
  virtual const std::unordered_map<std::string, karto::Pose2> & GetSensorNominalOffsets() const;

private:
  // Lazily create (on first sight of a sensor name) the live extrinsic parameter block for
  // that sensor, anchored to its nominal (TF-derived) offset with a Gaussian prior residual.
  // Caller must hold nodes_mutex_.
  void GetOrCreateSensorOffset(const std::string & sensorName, const karto::Pose2 & nominalOffset);
  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // ceres
  ceres::Solver::Options options_;
  ceres::Problem::Options options_problem_;
  ceres::LossFunction * loss_function_;
  ceres::Problem * problem_;
  ceres::LocalParameterization * angle_local_parameterization_;
  bool was_constant_set_, debug_logging_;

  // graph
  std::unordered_map<int, Eigen::Vector3d> * nodes_;
  // Value is a vector, not a single id: with extrinsic calibration enabled, one edge can own
  // two residual blocks (the extrinsic-aware/legacy pose residual, plus an independent
  // odometry residual -- see AddConstraint). RemoveConstraint removes all of them.
  std::unordered_map<size_t, std::vector<ceres::ResidualBlockId>> * blocks_;
  std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
  boost::mutex nodes_mutex_;

  // sensor extrinsic (lidar<->base_link) online joint calibration; guarded by nodes_mutex_,
  // like every other member of the graph -- deliberately not a second mutex, see AddConstraint.
  std::unordered_map<std::string, Eigen::Vector3d> * sensor_offsets_;
  std::unordered_set<std::string> * sensor_offsets_initialized_;
  std::unordered_map<std::string, Eigen::Matrix3d> * sensor_offset_covariances_;
  std::unordered_map<std::string, karto::Pose2> sensor_offset_corrections_;
  // Frozen nominal offset each sensor's prior was anchored to, set once in
  // GetOrCreateSensorOffset and never updated -- used to publish a residual-vs-nominal debug
  // view instead of the raw base_link-frame estimate.
  std::unordered_map<std::string, karto::Pose2> sensor_nominal_offsets_;
  bool optimize_sensor_extrinsics_;
  double extrinsic_prior_stddev_xy_, extrinsic_prior_stddev_yaw_;
  // Independent robot-frame (odometry) residual added alongside the sensor-frame one -- this is
  // what actually makes the extrinsic identifiable (AX=XB); see PLAN.md addendum.
  double odometry_edge_stddev_xy_, odometry_edge_stddev_yaw_;

  // ros
  rclcpp::Node::SharedPtr node_;
};

}  // namespace solver_plugins

#endif  // SOLVERS__CERES_SOLVER_HPP_
