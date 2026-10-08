/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include <algorithm>
#include <unordered_map>
#include <string>
#include <utility>
#include "ceres_solver.hpp"

namespace solver_plugins
{

/*****************************************************************************/
CeresSolver::CeresSolver()
: nodes_(new std::unordered_map<int, Eigen::Vector3d>()),
  blocks_(new std::unordered_map<std::size_t,
    std::vector<ceres::ResidualBlockId>>()),
  sensor_offsets_(new std::unordered_map<std::string, Eigen::Vector3d>()),
  sensor_offsets_initialized_(new std::unordered_set<std::string>()),
  sensor_offset_covariances_(new std::unordered_map<std::string, Eigen::Matrix3d>()),
  problem_(NULL), was_constant_set_(false)
/*****************************************************************************/
{
}

/*****************************************************************************/
void CeresSolver::Configure(rclcpp::Node::SharedPtr node)
/*****************************************************************************/
{
  node_ = node;

  std::string solver_type, preconditioner_type, dogleg_type,
    trust_strategy, loss_fn, mode;
  solver_type = node->declare_parameter("ceres_linear_solver",
      std::string("SPARSE_NORMAL_CHOLESKY"));
  preconditioner_type = node->declare_parameter("ceres_preconditioner",
      std::string("JACOBI"));
  dogleg_type = node->declare_parameter("ceres_dogleg_type",
      std::string("TRADITIONAL_DOGLEG"));
  trust_strategy = node->declare_parameter("ceres_trust_strategy",
      std::string("LM"));
  loss_fn = node->declare_parameter("ceres_loss_function",
      std::string("None"));
  mode = node->declare_parameter("mode", std::string("mapping"));
  debug_logging_ = node->get_parameter("debug_logging").as_bool();

  optimize_sensor_extrinsics_ = node->declare_parameter(
    "optimize_sensor_extrinsics", false);
  extrinsic_prior_stddev_xy_ = node->declare_parameter(
    "extrinsic_prior_stddev_xy", 0.02);
  extrinsic_prior_stddev_yaw_ = node->declare_parameter(
    "extrinsic_prior_stddev_yaw", 0.02);
  // Hard limits on how far the estimate may move from the nominal (TF) mount, per component:
  // Ceres bounds, so the calibration can never run off, and the clamp on seeds.
  // optimize_sensor_extrinsics puts the extrinsic into the graph (a calibrated mount, seedable);
  // these decide whether solves keep learning its x/y and its yaw or leave them as they are.
  // x/y are only observable through turns between nodes, yaw also from straight driving.
  estimate_sensor_extrinsic_xy_ = node->declare_parameter("estimate_sensor_extrinsic_xy", true);
  estimate_sensor_extrinsic_yaw_ =
    node->declare_parameter("estimate_sensor_extrinsic_yaw", true);
  extrinsic_max_correction_xy_ = node->declare_parameter(
    "extrinsic_max_correction_xy", 0.05);
  extrinsic_max_correction_yaw_ = node->declare_parameter(
    "extrinsic_max_correction_yaw", 0.035);
  odometry_edge_stddev_xy_ = node->declare_parameter(
    "odometry_edge_stddev_xy", 0.03);
  odometry_edge_stddev_yaw_ = node->declare_parameter(
    "odometry_edge_stddev_yaw", 0.02);
  // Without extrinsic calibration the graph otherwise holds only scan-match edges, so good
  // wheel+IMU odometry has no say and the periodic solve has nothing to correct
  // (REFLECTOR_SLAM.md §5.5, §7.1).
  use_odometry_edges_ = node->declare_parameter("use_odometry_edges", false);
  // Compare odometry edges' translation in the edge's midpoint frame (OdometryMidpointErrorTerm):
  // a turn-rate error in odometry then no longer biases the direction, nor the calibrated yaw.
  odometry_edge_midpoint_ = node->declare_parameter("odometry_edge_midpoint", false);

  corrections_.clear();
  first_node_ = nodes_->end();

  // formulate problem
  angle_local_parameterization_ = AngleLocalParameterization::Create();

  // choose loss function default squared loss (NULL)
  loss_function_ = NULL;
  if (loss_fn == "HuberLoss") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using HuberLoss loss function.");
    loss_function_ = new ceres::HuberLoss(0.7);
  } else if (loss_fn == "CauchyLoss") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using CauchyLoss loss function.");
    loss_function_ = new ceres::CauchyLoss(0.7);
  }

  // choose linear solver default CHOL
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (solver_type == "SPARSE_SCHUR") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using SPARSE_SCHUR solver.");
    options_.linear_solver_type = ceres::SPARSE_SCHUR;
  } else if (solver_type == "ITERATIVE_SCHUR") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using ITERATIVE_SCHUR solver.");
    options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
  } else if (solver_type == "CGNR") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using CGNR solver.");
    options_.linear_solver_type = ceres::CGNR;
  }

  // choose preconditioner default Jacobi
  options_.preconditioner_type = ceres::JACOBI;
  if (preconditioner_type == "IDENTITY") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using IDENTITY preconditioner.");
    options_.preconditioner_type = ceres::IDENTITY;
  } else if (preconditioner_type == "SCHUR_JACOBI") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using SCHUR_JACOBI preconditioner.");
    options_.preconditioner_type = ceres::SCHUR_JACOBI;
  }

  if (options_.preconditioner_type == ceres::CLUSTER_JACOBI ||
    options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
  {
    // default canonical view is O(n^2) which is unacceptable for
    // problems of this size
    options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
  }

  // choose trust region strategy default LM
  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  if (trust_strategy == "DOGLEG") {
    RCLCPP_INFO(node_->get_logger(),
      "CeresSolver: Using DOGLEG trust region strategy.");
    options_.trust_region_strategy_type = ceres::DOGLEG;
  }

  // choose dogleg type default traditional
  if (options_.trust_region_strategy_type == ceres::DOGLEG) {
    options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
    if (dogleg_type == "SUBSPACE_DOGLEG") {
      RCLCPP_INFO(node_->get_logger(),
        "CeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
      options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
    }
  }

  // a typical ros map is 5cm, this is 0.001, 50x the resolution
  options_.function_tolerance = 1e-3;
  options_.gradient_tolerance = 1e-6;
  options_.parameter_tolerance = 1e-3;

  options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps =
    options_.max_num_consecutive_invalid_steps;
  options_.num_threads = node->declare_parameter("ceres_num_threads", 8);
  options_.use_nonmonotonic_steps = true;
  options_.jacobi_scaling = true;

  options_.min_relative_decrease = 1e-3;

  options_.initial_trust_region_radius = 1e4;
  options_.max_trust_region_radius = 1e8;
  options_.min_trust_region_radius = 1e-16;

  options_.min_lm_diagonal = 1e-6;
  options_.max_lm_diagonal = 1e32;

  if (options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY) {
    options_.dynamic_sparsity = true;
  }

  if (mode == std::string("localization")) {
    // doubles the memory footprint, but lets us remove contraints faster
    options_problem_.enable_fast_removal = true;
  }

  // we do not want the problem definition to own these objects, otherwise they get
  // deleted along with the problem 
  options_problem_.loss_function_ownership = ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  problem_ = new ceres::Problem(options_problem_);
}

/*****************************************************************************/
CeresSolver::~CeresSolver()
/*****************************************************************************/
{
  if (loss_function_ != NULL) {
    delete loss_function_;
  }
  if (nodes_ != NULL) {
    delete nodes_;
  }
  if (blocks_ != NULL) {
    delete blocks_;
  }
  if (sensor_offsets_ != NULL) {
    delete sensor_offsets_;
  }
  if (sensor_offsets_initialized_ != NULL) {
    delete sensor_offsets_initialized_;
  }
  if (sensor_offset_covariances_ != NULL) {
    delete sensor_offset_covariances_;
  }
  if (problem_ != NULL) {
    delete problem_;
  }
}

/*****************************************************************************/
void CeresSolver::Compute()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (nodes_->size() == 0) {
    RCLCPP_WARN(node_->get_logger(),
      "CeresSolver: Ceres was called when there are no nodes."
      " This shouldn't happen.");
    return;
  }

  // populate contraint for static initial pose
  if (!was_constant_set_ && first_node_ != nodes_->end() &&
      problem_->HasParameterBlock(&first_node_->second(0)) &&
      problem_->HasParameterBlock(&first_node_->second(1)) &&
      problem_->HasParameterBlock(&first_node_->second(2))) {
    RCLCPP_DEBUG(node_->get_logger(),
      "CeresSolver: Setting first node as a constant pose:"
      "%0.2f, %0.2f, %0.2f.", first_node_->second(0),
      first_node_->second(1), first_node_->second(2));
    problem_->SetParameterBlockConstant(&first_node_->second(0));
    problem_->SetParameterBlockConstant(&first_node_->second(1));
    problem_->SetParameterBlockConstant(&first_node_->second(2));
    was_constant_set_ = !was_constant_set_;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);
  if (debug_logging_) {
    std::cout << summary.FullReport() << '\n';
  }

  if (!summary.IsSolutionUsable()) {
    RCLCPP_WARN(node_->get_logger(), "CeresSolver: "
      "Ceres could not find a usable solution to optimize.");
    return;
  }

  // store corrected poses
  if (!corrections_.empty()) {
    corrections_.clear();
  }
  corrections_.reserve(nodes_->size());
  karto::Pose2 pose;
  ConstGraphIterator iter = nodes_->begin();
  for (iter; iter != nodes_->end(); ++iter) {
    pose.SetX(iter->second(0));
    pose.SetY(iter->second(1));
    pose.SetHeading(iter->second(2));
    corrections_.push_back(std::make_pair(iter->first, pose));
  }

  // store the live sensor extrinsic estimate(s), and their marginal covariance for the RViz
  // debug publisher -- both empty/unpopulated if extrinsic calibration is disabled.
  sensor_offset_corrections_.clear();
  if (optimize_sensor_extrinsics_) {
    for (auto & kv : *sensor_offsets_) {
      sensor_offset_corrections_[kv.first] =
        karto::Pose2(kv.second(0), kv.second(1), kv.second(2));
    }

    // Over the learned components only (constant blocks have no covariance): a frozen one gets
    // variance 0, and a fully frozen estimate keeps the covariance it was last learned with.
    std::vector<int> free;
    if (estimate_sensor_extrinsic_xy_) {
      free = {0, 1};
    }
    if (estimate_sensor_extrinsic_yaw_) {
      free.push_back(2);
    }
    ceres::Covariance::Options cov_options;
    for (auto & kv : *sensor_offsets_) {
      if (free.empty()) {
        break;
      }
      Eigen::Vector3d & ext = kv.second;
      std::vector<const double *> blocks;
      for (const int i : free) {
        blocks.push_back(&ext(i));
      }
      ceres::Covariance covariance(cov_options);
      if (covariance.Compute(blocks, problem_)) {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> part(
          free.size(), free.size());
        if (covariance.GetCovarianceMatrix(blocks, part.data())) {
          Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
          for (size_t a = 0; a < free.size(); ++a) {
            for (size_t b = 0; b < free.size(); ++b) {
              cov(free[a], free[b]) = part(a, b);
            }
          }
          (*sensor_offset_covariances_)[kv.first] = cov;
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "CeresSolver: Failed to compute marginal covariance for sensor extrinsic '%s'.",
          kv.first.c_str());
      }
    }
  }
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector & CeresSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void CeresSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void CeresSolver::Reset()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  corrections_.clear();
  was_constant_set_ = false;

  if (problem_) {
    // Note that this also frees anything the problem owns (i.e. local parameterization, cost
    // function)
    delete problem_;
  }

  if (nodes_) {
    delete nodes_;
  }

  if (blocks_) {
    delete blocks_;
  }

  if (sensor_offsets_) {
    delete sensor_offsets_;
  }

  if (sensor_offsets_initialized_) {
    delete sensor_offsets_initialized_;
  }

  if (sensor_offset_covariances_) {
    delete sensor_offset_covariances_;
  }

  nodes_ = new std::unordered_map<int, Eigen::Vector3d>();
  blocks_ = new std::unordered_map<std::size_t, std::vector<ceres::ResidualBlockId>>();
  sensor_offsets_ = new std::unordered_map<std::string, Eigen::Vector3d>();
  sensor_offsets_initialized_ = new std::unordered_set<std::string>();
  sensor_offset_covariances_ = new std::unordered_map<std::string, Eigen::Matrix3d>();
  sensor_offset_corrections_.clear();
  problem_ = new ceres::Problem(options_problem_);
  first_node_ = nodes_->end();

  angle_local_parameterization_ = AngleLocalParameterization::Create();
}

/*****************************************************************************/
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex)
/*****************************************************************************/
{
  // store nodes
  if (!pVertex) {
    return;
  }

  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());

  const int id = pVertex->GetObject()->GetUniqueId();

  boost::mutex::scoped_lock lock(nodes_mutex_);
  nodes_->insert(std::pair<int, Eigen::Vector3d>(id, pose2d));

  if (nodes_->size() == 1) {
    first_node_ = nodes_->find(id);
  }
}

/*****************************************************************************/
void CeresSolver::GetOrCreateSensorOffset(
  const std::string & sensorName, const karto::Pose2 & nominalOffset)
/*****************************************************************************/
{
  // caller (AddConstraint) already holds nodes_mutex_
  if (sensor_offsets_initialized_->count(sensorName)) {
    return;
  }

  // The laser's offset is the nominal only until the first solve feeds the estimate back
  // (MapperGraph::CorrectPoses), so a sensor seen before (InitializeSensorOffset, or before a
  // Reset) keeps the nominal it was first given.
  const karto::Pose2 nominal =
    sensor_nominal_offsets_.emplace(sensorName, nominalOffset).first->second;
  const karto::Pose2 start = ClampedSensorOffset(nominal, sensor_offset_seed_);
  (*sensor_offsets_)[sensorName] =
    Eigen::Vector3d(start.GetX(), start.GetY(), start.GetHeading());
  Eigen::Vector3d & ref = (*sensor_offsets_)[sensorName];
  sensor_offset_corrections_[sensorName] = start;
  RCLCPP_INFO(node_->get_logger(),
    "CeresSolver: extrinsic of '%s' starts at x %.4f y %.4f yaw %.3f deg (nominal x %.4f y %.4f "
    "yaw %.3f deg), x/y %s, yaw %s", sensorName.c_str(), start.GetX(), start.GetY(),
    start.GetHeading() * 180.0 / M_PI, nominal.GetX(), nominal.GetY(),
    nominal.GetHeading() * 180.0 / M_PI, estimate_sensor_extrinsic_xy_ ? "learned" : "fixed",
    estimate_sensor_extrinsic_yaw_ ? "learned" : "fixed");

  // AddResidualBlock is what implicitly creates the (x_e, y_e, yaw_e) parameter blocks in the
  // Ceres problem -- SetParameterization must come after, not before, or Ceres doesn't know
  // about the block yet. The prior stays on the nominal, whatever the start: its stddev is the
  // mount's tolerance, and a bad seed must not pull the estimate for good.
  ceres::CostFunction * prior = ExtrinsicPriorErrorTerm::Create(
    nominal.GetX(), nominal.GetY(), nominal.GetHeading(), extrinsic_prior_stddev_xy_,
    extrinsic_prior_stddev_yaw_);
  problem_->AddResidualBlock(prior, nullptr /* no robust loss on the prior */,
    &ref(0), &ref(1), &ref(2));
  problem_->SetParameterization(&ref(2), angle_local_parameterization_);
  ApplySensorOffsetFreedom(ref);

  problem_->SetParameterLowerBound(&ref(0), 0, nominal.GetX() - extrinsic_max_correction_xy_);
  problem_->SetParameterUpperBound(&ref(0), 0, nominal.GetX() + extrinsic_max_correction_xy_);
  problem_->SetParameterLowerBound(&ref(1), 0, nominal.GetY() - extrinsic_max_correction_xy_);
  problem_->SetParameterUpperBound(&ref(1), 0, nominal.GetY() + extrinsic_max_correction_xy_);
  // The yaw parameterization wraps at +-pi, so a yaw box only works when it does not reach it.
  if (std::abs(nominal.GetHeading()) + extrinsic_max_correction_yaw_ < M_PI) {
    problem_->SetParameterLowerBound(
      &ref(2), 0, nominal.GetHeading() - extrinsic_max_correction_yaw_);
    problem_->SetParameterUpperBound(
      &ref(2), 0, nominal.GetHeading() + extrinsic_max_correction_yaw_);
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "CeresSolver: nominal lidar yaw %.3f is within extrinsic_max_correction_yaw of +-pi; "
      "the extrinsic yaw is not bounded.", nominal.GetHeading());
  }

  sensor_offsets_initialized_->insert(sensorName);
}

/*****************************************************************************/
karto::Pose2 CeresSolver::ClampedSensorOffset(
  const karto::Pose2 & nominal, const karto::Pose2 & correction) const
/*****************************************************************************/
{
  const karto::Pose2 offset = karto::Transform(nominal).TransformPose(correction);
  const double max_xy = extrinsic_max_correction_xy_;
  const double max_yaw = extrinsic_max_correction_yaw_;
  return karto::Pose2(
    nominal.GetX() + std::clamp(offset.GetX() - nominal.GetX(), -max_xy, max_xy),
    nominal.GetY() + std::clamp(offset.GetY() - nominal.GetY(), -max_xy, max_xy),
    karto::math::NormalizeAngle(nominal.GetHeading() + std::clamp(
      karto::math::NormalizeAngle(offset.GetHeading() - nominal.GetHeading()), -max_yaw,
      max_yaw)));
}

/*****************************************************************************/
kt_bool CeresSolver::SetSensorOffsetCorrection(const karto::Pose2 & correction)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  sensor_offset_seed_ = correction;
  if (!optimize_sensor_extrinsics_) {
    return false;
  }
  kt_bool changed = false;
  for (auto & kv : *sensor_offsets_) {
    const karto::Pose2 offset =
      ClampedSensorOffset(sensor_nominal_offsets_.at(kv.first), correction);
    kv.second = Eigen::Vector3d(offset.GetX(), offset.GetY(), offset.GetHeading());
    sensor_offset_corrections_[kv.first] = offset;
    // computed for the old estimate; the next solve recomputes it
    sensor_offset_covariances_->erase(kv.first);
    changed = true;
  }
  return changed;
}

/*****************************************************************************/
kt_bool CeresSolver::SetSensorOffsetEstimation(
  karto::ScanSolver::SensorOffsetComponent component, kt_bool estimate)
/*****************************************************************************/
{
  if (!optimize_sensor_extrinsics_) {
    return false;
  }
  boost::mutex::scoped_lock lock(nodes_mutex_);
  (component == SensorOffsetXY ? estimate_sensor_extrinsic_xy_ : estimate_sensor_extrinsic_yaw_) =
    estimate;
  for (auto & kv : *sensor_offsets_) {
    ApplySensorOffsetFreedom(kv.second);
  }
  return true;
}

/*****************************************************************************/
kt_bool CeresSolver::GetSensorOffsetEstimation(
  karto::ScanSolver::SensorOffsetComponent component) const
/*****************************************************************************/
{
  return optimize_sensor_extrinsics_ &&
         (component == SensorOffsetXY ? estimate_sensor_extrinsic_xy_ :
         estimate_sensor_extrinsic_yaw_);
}

/*****************************************************************************/
void CeresSolver::ApplySensorOffsetFreedom(Eigen::Vector3d & offset)
/*****************************************************************************/
{
  // caller holds nodes_mutex_
  const bool free[3] = {
    estimate_sensor_extrinsic_xy_, estimate_sensor_extrinsic_xy_, estimate_sensor_extrinsic_yaw_};
  for (int i = 0; i < 3; ++i) {
    if (free[i]) {
      problem_->SetParameterBlockVariable(&offset(i));
    } else {
      problem_->SetParameterBlockConstant(&offset(i));
    }
  }
}

/*****************************************************************************/
kt_bool CeresSolver::InitializeSensorOffset(
  const std::string & sensorName, const karto::Pose2 & nominal, karto::Pose2 & estimate)
/*****************************************************************************/
{
  if (!optimize_sensor_extrinsics_) {
    return false;
  }
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GetOrCreateSensorOffset(sensorName, nominal);
  estimate = sensor_offset_corrections_.at(sensorName);
  return true;
}

/*****************************************************************************/
ceres::CostFunction * CeresSolver::OdometryCostFunction(
  double x, double y, double yaw, const Eigen::Matrix3d & sqrt_information) const
/*****************************************************************************/
{
  if (odometry_edge_midpoint_) {
    return OdometryMidpointErrorTerm::Create(x, y, yaw, sqrt_information);
  }
  return PoseGraph2dErrorTerm::Create(x, y, yaw, sqrt_information);
}

/*****************************************************************************/
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge)
/*****************************************************************************/
{
  // get IDs in graph for this edge
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (!pEdge) {
    return;
  }

  const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
  GraphIterator node1it = nodes_->find(node1);
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
  GraphIterator node2it = nodes_->find(node2);

  if (node1it == nodes_->end() ||
    node2it == nodes_->end() || node1it == node2it)
  {
    RCLCPP_WARN(node_->get_logger(),
      "CeresSolver: Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo * pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());

  std::vector<ceres::ResidualBlockId> edge_blocks;

  if (optimize_sensor_extrinsics_ && pLinkInfo->HasSensorFrameData()) {
    // joint lidar<->base_link extrinsic calibration path: the measurement was made in the
    // sensor frame, and the extrinsic is a live parameter block shared by every edge from
    // this sensor, anchored to its nominal (TF-derived) value by a Gaussian prior.
    const std::string sensorName =
      pEdge->GetSource()->GetObject()->GetSensorName().ToString();
    const karto::Pose2 nominalOffset =
      pEdge->GetSource()->GetObject()->GetLaserRangeFinder()->GetOffsetPose();
    GetOrCreateSensorOffset(sensorName, nominalOffset);
    Eigen::Vector3d & ext = (*sensor_offsets_)[sensorName];

    karto::Pose2 sdiff = pLinkInfo->GetSensorPoseDifference();
    Eigen::Vector3d spose2d(sdiff.GetX(), sdiff.GetY(), sdiff.GetHeading());

    karto::Matrix3 sprecision = pLinkInfo->GetSensorCovariance().Inverse();
    Eigen::Matrix3d sinformation;
    sinformation(0, 0) = sprecision(0, 0);
    sinformation(0, 1) = sinformation(1, 0) = sprecision(0, 1);
    sinformation(0, 2) = sinformation(2, 0) = sprecision(0, 2);
    sinformation(1, 1) = sprecision(1, 1);
    sinformation(1, 2) = sinformation(2, 1) = sprecision(1, 2);
    sinformation(2, 2) = sprecision(2, 2);
    Eigen::Matrix3d ssqrt_information = sinformation.llt().matrixU();

    ceres::CostFunction * cost_function = SensorExtrinsicPoseGraph2dErrorTerm::Create(
      spose2d(0), spose2d(1), spose2d(2), ssqrt_information);
    edge_blocks.push_back(problem_->AddResidualBlock(
      cost_function, loss_function_,
      &node1it->second(0), &node1it->second(1), &node1it->second(2),
      &node2it->second(0), &node2it->second(1), &node2it->second(2),
      &ext(0), &ext(1), &ext(2)));
    problem_->SetParameterization(&node1it->second(2),
      angle_local_parameterization_);
    problem_->SetParameterization(&node2it->second(2),
      angle_local_parameterization_);

    if (pLinkInfo->HasOdomFrameData()) {
      // Independent robot-frame measurement (fused wheel+IMU odometry, not scan matching).
      // Without this, the extrinsic-aware residual above has an exact SE(2) conjugation
      // symmetry in `ext` (for any ext there's a compensating set of node poses that zeroes
      // every sensor-frame residual identically) -- this is the AX=XB structure that actually
      // makes `ext` identifiable. See PLAN.md addendum for the full derivation.
      karto::Pose2 odiff = pLinkInfo->GetOdomPoseDifference();
      Eigen::Vector3d opose2d(odiff.GetX(), odiff.GetY(), odiff.GetHeading());

      Eigen::Matrix3d odom_sqrt_information = Eigen::Matrix3d::Zero();
      odom_sqrt_information(0, 0) = 1.0 / odometry_edge_stddev_xy_;
      odom_sqrt_information(1, 1) = 1.0 / odometry_edge_stddev_xy_;
      odom_sqrt_information(2, 2) = 1.0 / odometry_edge_stddev_yaw_;

      ceres::CostFunction * odom_cost_function = OdometryCostFunction(
        opose2d(0), opose2d(1), opose2d(2), odom_sqrt_information);
      edge_blocks.push_back(problem_->AddResidualBlock(
        odom_cost_function, loss_function_,
        &node1it->second(0), &node1it->second(1), &node1it->second(2),
        &node2it->second(0), &node2it->second(1), &node2it->second(2)));
      // node1/node2 yaw parameterization already set above for this same edge.
    }
  } else {
    // legacy path: robot-frame measurement, no extrinsic parameter (used whenever extrinsic
    // calibration is disabled, and as a fallback for edges loaded from a pose graph serialized
    // before this feature existed, which have no sensor-frame data).
    karto::Pose2 diff = pLinkInfo->GetPoseDifference();
    Eigen::Vector3d pose2d(diff.GetX(), diff.GetY(), diff.GetHeading());

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
    Eigen::Matrix3d information;
    information(0, 0) = precisionMatrix(0, 0);
    information(0, 1) = information(1, 0) = precisionMatrix(0, 1);
    information(0, 2) = information(2, 0) = precisionMatrix(0, 2);
    information(1, 1) = precisionMatrix(1, 1);
    information(1, 2) = information(2, 1) = precisionMatrix(1, 2);
    information(2, 2) = precisionMatrix(2, 2);
    Eigen::Matrix3d sqrt_information = information.llt().matrixU();

    // populate residual and parameterization for heading normalization
    ceres::CostFunction * cost_function = PoseGraph2dErrorTerm::Create(pose2d(0),
        pose2d(1), pose2d(2), sqrt_information);
    edge_blocks.push_back(problem_->AddResidualBlock(
      cost_function, loss_function_,
      &node1it->second(0), &node1it->second(1), &node1it->second(2),
      &node2it->second(0), &node2it->second(1), &node2it->second(2)));
    problem_->SetParameterization(&node1it->second(2),
      angle_local_parameterization_);
    problem_->SetParameterization(&node2it->second(2),
      angle_local_parameterization_);

    // Independent robot-frame odometry measurement, as in the extrinsic path above; Karto
    // attaches it to sequential edges only.
    if (use_odometry_edges_ && pLinkInfo->HasOdomFrameData()) {
      karto::Pose2 odiff = pLinkInfo->GetOdomPoseDifference();
      Eigen::Matrix3d odom_sqrt_information = Eigen::Matrix3d::Zero();
      odom_sqrt_information(0, 0) = 1.0 / odometry_edge_stddev_xy_;
      odom_sqrt_information(1, 1) = 1.0 / odometry_edge_stddev_xy_;
      odom_sqrt_information(2, 2) = 1.0 / odometry_edge_stddev_yaw_;
      ceres::CostFunction * odom_cost_function = OdometryCostFunction(
        odiff.GetX(), odiff.GetY(), odiff.GetHeading(), odom_sqrt_information);
      edge_blocks.push_back(problem_->AddResidualBlock(
        odom_cost_function, loss_function_,
        &node1it->second(0), &node1it->second(1), &node1it->second(2),
        &node2it->second(0), &node2it->second(1), &node2it->second(2)));
    }
  }

  blocks_->insert(std::pair<std::size_t, std::vector<ceres::ResidualBlockId>>(
      GetHash(node1, node2), edge_blocks));
}

/*****************************************************************************/
void CeresSolver::RemoveNode(kt_int32s id)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator nodeit = nodes_->find(id);
  if (nodeit != nodes_->end()) {
    if (problem_->HasParameterBlock(&nodeit->second(0)) &&
        problem_->HasParameterBlock(&nodeit->second(1)) &&
        problem_->HasParameterBlock(&nodeit->second(2)))
    {
      problem_->RemoveParameterBlock(&nodeit->second(0));
      problem_->RemoveParameterBlock(&nodeit->second(1));
      problem_->RemoveParameterBlock(&nodeit->second(2));
      RCLCPP_DEBUG(
        node_->get_logger(),
        "RemoveNode: Removed node id %d" ,nodeit->first);
    }
    else
    {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "RemoveNode: Missing parameter blocks for "
        "node id %d", nodeit->first);
    }
    nodes_->erase(nodeit);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "RemoveNode: Failed to find node matching id %i",
      (int)id);
  }
}

/*****************************************************************************/
void CeresSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  std::unordered_map<std::size_t, std::vector<ceres::ResidualBlockId>>::iterator it_a =
    blocks_->find(GetHash(sourceId, targetId));
  std::unordered_map<std::size_t, std::vector<ceres::ResidualBlockId>>::iterator it_b =
    blocks_->find(GetHash(targetId, sourceId));
  if (it_a != blocks_->end()) {
    for (const auto & block : it_a->second) {
      problem_->RemoveResidualBlock(block);
    }
    blocks_->erase(it_a);
  } else if (it_b != blocks_->end()) {
    for (const auto & block : it_b->second) {
      problem_->RemoveResidualBlock(block);
    }
    blocks_->erase(it_b);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "RemoveConstraint: Failed to find residual block for %i %i",
      (int)sourceId, (int)targetId);
  }
}

/*****************************************************************************/
void CeresSolver::ModifyNode(const int & unique_id, Eigen::Vector3d pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    double yaw_init = it->second(2);
    it->second = pose;
    it->second(2) += yaw_init;
  }
}

/*****************************************************************************/
void CeresSolver::GetNodeOrientation(const int & unique_id, double & pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  GraphIterator it = nodes_->find(unique_id);
  if (it != nodes_->end()) {
    pose = it->second(2);
  }
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Vector3d> * CeresSolver::getGraph()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  return nodes_;
}

/*****************************************************************************/
const std::unordered_map<std::string, karto::Pose2> &
CeresSolver::GetSensorOffsetCorrections() const
/*****************************************************************************/
{
  return sensor_offset_corrections_;
}

/*****************************************************************************/
const std::unordered_map<std::string, Eigen::Matrix3d> &
CeresSolver::GetSensorOffsetCovariances() const
/*****************************************************************************/
{
  return *sensor_offset_covariances_;
}

/*****************************************************************************/
const std::unordered_map<std::string, karto::Pose2> &
CeresSolver::GetSensorNominalOffsets() const
/*****************************************************************************/
{
  return sensor_nominal_offsets_;
}

}  // namespace solver_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solver_plugins::CeresSolver, karto::ScanSolver)
