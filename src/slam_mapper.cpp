/*
 * slam_mapper
 * Copyright (c) 2018, Simbe Robotics
 * Copyright (c) 2018, Steve Macenski
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include <memory>
#include "slam_toolbox/slam_mapper.hpp"

namespace mapper_utils
{

/*****************************************************************************/
SMapper::SMapper()
/*****************************************************************************/
{
  mapper_ = std::make_unique<karto::Mapper>();
}

/*****************************************************************************/
SMapper::~SMapper()
/*****************************************************************************/
{
  mapper_.reset();
}

/*****************************************************************************/
karto::Mapper * SMapper::getMapper()
/*****************************************************************************/
{
  return mapper_.get();
}

/*****************************************************************************/
void SMapper::setMapper(karto::Mapper * mapper)
/*****************************************************************************/
{
  mapper_.reset(mapper);
}

/*****************************************************************************/
void SMapper::clearLocalizationBuffer()
/*****************************************************************************/
{
  mapper_->ClearLocalizationBuffer();
}

/*****************************************************************************/
karto::OccupancyGrid * SMapper::getOccupancyGrid(const double & resolution)
/*****************************************************************************/
{
  karto::OccupancyGrid * occ_grid = nullptr;
  return karto::OccupancyGrid::CreateFromScans(
    mapper_->GetAllProcessedScans(),
    resolution, (kt_int32u)mapper_->getParamMinPassThrough(), (kt_double)mapper_->getParamOccupancyThreshold());
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2 & pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
}

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform & pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
}

/*****************************************************************************/
void SMapper::configure(const rclcpp::Node::SharedPtr & node)
/*****************************************************************************/
{
  bool use_scan_matching = true;
  if (!node->has_parameter("use_scan_matching")) {
    node->declare_parameter("use_scan_matching", use_scan_matching);
  }
  node->get_parameter("use_scan_matching", use_scan_matching);
  mapper_->setParamUseScanMatching(use_scan_matching);

  bool use_scan_barycenter = true;
  if (!node->has_parameter("use_scan_barycenter")) {
    node->declare_parameter("use_scan_barycenter", use_scan_barycenter);
  }
  node->get_parameter("use_scan_barycenter", use_scan_barycenter);
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  double minimum_travel_distance = 0.5;
  if (!node->has_parameter("minimum_travel_distance")) {
    node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
  }
  node->get_parameter("minimum_travel_distance", minimum_travel_distance);
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  double minimum_travel_heading = 0.5;
  if (!node->has_parameter("minimum_travel_heading")) {
    node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
  }
  node->get_parameter("minimum_travel_heading", minimum_travel_heading);
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  double minimum_time_interval = 0.5;
  if (!node->has_parameter("minimum_time_interval")) {
    node->declare_parameter("minimum_time_interval", minimum_time_interval);
  }
  node->get_parameter("minimum_time_interval", minimum_time_interval);
  mapper_->setParamMinimumTimeInterval(minimum_time_interval);

  int scan_buffer_size = 10;
  if (!node->has_parameter("scan_buffer_size")) {
    node->declare_parameter("scan_buffer_size", scan_buffer_size);
  }
  node->get_parameter("scan_buffer_size", scan_buffer_size);
  if (scan_buffer_size <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_size to be a value smaller than zero,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_size = 10;
  }
  mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance = 10;
  if (!node->has_parameter("scan_buffer_maximum_scan_distance")) {
    node->declare_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  }
  node->get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance);
  if (math::Square(scan_buffer_maximum_scan_distance) <= 1e-06) {
    RCLCPP_WARN(node->get_logger(),
      "You've set scan_buffer_maximum_scan_distance to be a value whose square is smaller than 1e-06,"
      "this isn't allowed so it will be set to default value 10.");
    scan_buffer_maximum_scan_distance = 10;
  }
  mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  double link_match_minimum_response_fine = 0.1;
  if (!node->has_parameter("link_match_minimum_response_fine")) {
    node->declare_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  }
  node->get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine);
  mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  double link_scan_maximum_distance = 1.5;
  if (!node->has_parameter("link_scan_maximum_distance")) {
    node->declare_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  }
  node->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance);
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  double loop_search_maximum_distance = 3.0;
  if (!node->has_parameter("loop_search_maximum_distance")) {
    node->declare_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  }
  node->get_parameter("loop_search_maximum_distance", loop_search_maximum_distance);
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  bool do_loop_closing = true;
  if (!node->has_parameter("do_loop_closing")) {
    node->declare_parameter("do_loop_closing", do_loop_closing);
  }
  node->get_parameter("do_loop_closing", do_loop_closing);
  mapper_->setParamDoLoopClosing(do_loop_closing);

  int loop_match_minimum_chain_size = 10;
  if (!node->has_parameter("loop_match_minimum_chain_size")) {
    node->declare_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  }
  node->get_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size);
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  bool enable_periodic_pose_graph_solve = false;
  if (!node->has_parameter("enable_periodic_pose_graph_solve")) {
    node->declare_parameter(
      "enable_periodic_pose_graph_solve", enable_periodic_pose_graph_solve);
  }
  node->get_parameter("enable_periodic_pose_graph_solve", enable_periodic_pose_graph_solve);
  mapper_->setParamEnablePeriodicPoseGraphSolve(enable_periodic_pose_graph_solve);

  int periodic_solve_interval_nodes = 20;
  if (!node->has_parameter("periodic_solve_interval_nodes")) {
    node->declare_parameter("periodic_solve_interval_nodes", periodic_solve_interval_nodes);
  }
  node->get_parameter("periodic_solve_interval_nodes", periodic_solve_interval_nodes);
  mapper_->setParamPeriodicSolveIntervalNodes(periodic_solve_interval_nodes);

  double loop_match_maximum_variance_coarse = 3.0;
  if (!node->has_parameter("loop_match_maximum_variance_coarse")) {
    node->declare_parameter(
      "loop_match_maximum_variance_coarse",
      loop_match_maximum_variance_coarse);
  }
  node->get_parameter("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse);
  mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse = 0.35;
  if (!node->has_parameter("loop_match_minimum_response_coarse")) {
    node->declare_parameter(
      "loop_match_minimum_response_coarse",
      loop_match_minimum_response_coarse);
  }
  node->get_parameter("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse);
  mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine = 0.45;
  if (!node->has_parameter("loop_match_minimum_response_fine")) {
    node->declare_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  }
  node->get_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine);
  mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  double correlation_search_space_dimension = 0.5;
  if (!node->has_parameter("correlation_search_space_dimension")) {
    node->declare_parameter(
      "correlation_search_space_dimension",
      correlation_search_space_dimension);
  }
  node->get_parameter("correlation_search_space_dimension", correlation_search_space_dimension);
  if (correlation_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 0.5.");
    correlation_search_space_dimension = 0.5;
  }
  mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution = 0.01;
  if (!node->has_parameter("correlation_search_space_resolution")) {
    node->declare_parameter(
      "correlation_search_space_resolution",
      correlation_search_space_resolution);
  }
  node->get_parameter("correlation_search_space_resolution", correlation_search_space_resolution);
  if (correlation_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.01.");
    correlation_search_space_resolution = 0.01;
  }
  mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation = 0.1;
  if (!node->has_parameter("correlation_search_space_smear_deviation")) {
    node->declare_parameter(
      "correlation_search_space_smear_deviation",
      correlation_search_space_smear_deviation);
  }
  node->get_parameter(
    "correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation);
  if (correlation_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set correlation_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.1.");
    correlation_search_space_smear_deviation = 0.1;
  }
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension = 8.0;
  if (!node->has_parameter("loop_search_space_dimension")) {
    node->declare_parameter("loop_search_space_dimension", loop_search_space_dimension);
  }
  node->get_parameter("loop_search_space_dimension", loop_search_space_dimension);
  if (loop_search_space_dimension <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_dimension to be negative,"
      "this isn't allowed so it will be set to default value 8.0.");
    loop_search_space_dimension = 8.0;
  }
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  double loop_search_space_resolution = 0.05;
  if (!node->has_parameter("loop_search_space_resolution")) {
    node->declare_parameter("loop_search_space_resolution", loop_search_space_resolution);
  }
  node->get_parameter("loop_search_space_resolution", loop_search_space_resolution);
  if (loop_search_space_resolution <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_resolution to be negative,"
      "this isn't allowed so it will be set to default value 0.05.");
    loop_search_space_resolution = 0.05;
  }
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  double loop_search_space_smear_deviation = 0.03;
  if (!node->has_parameter("loop_search_space_smear_deviation")) {
    node->declare_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  }
  node->get_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation);
  if (loop_search_space_smear_deviation <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "You've set loop_search_space_smear_deviation to be negative,"
      "this isn't allowed so it will be set to default value 0.03.");
    loop_search_space_smear_deviation = 0.03;
  }
  mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  double distance_variance_penalty = 0.5;
  if (!node->has_parameter("distance_variance_penalty")) {
    node->declare_parameter("distance_variance_penalty", distance_variance_penalty);
  }
  node->get_parameter("distance_variance_penalty", distance_variance_penalty);
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty = 1.0;
  if (!node->has_parameter("angle_variance_penalty")) {
    node->declare_parameter("angle_variance_penalty", angle_variance_penalty);
  }
  node->get_parameter("angle_variance_penalty", angle_variance_penalty);
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset = 0.00349;
  if (!node->has_parameter("fine_search_angle_offset")) {
    node->declare_parameter("fine_search_angle_offset", fine_search_angle_offset);
  }
  node->get_parameter("fine_search_angle_offset", fine_search_angle_offset);
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset = 0.349;
  if (!node->has_parameter("coarse_search_angle_offset")) {
    node->declare_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  }
  node->get_parameter("coarse_search_angle_offset", coarse_search_angle_offset);
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution = 0.0349;
  if (!node->has_parameter("coarse_angle_resolution")) {
    node->declare_parameter("coarse_angle_resolution", coarse_angle_resolution);
  }
  node->get_parameter("coarse_angle_resolution", coarse_angle_resolution);
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty = 0.9;
  if (!node->has_parameter("minimum_angle_penalty")) {
    node->declare_parameter("minimum_angle_penalty", minimum_angle_penalty);
  }
  node->get_parameter("minimum_angle_penalty", minimum_angle_penalty);
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty = 0.05;
  if (!node->has_parameter("minimum_distance_penalty")) {
    node->declare_parameter("minimum_distance_penalty", minimum_distance_penalty);
  }
  node->get_parameter("minimum_distance_penalty", minimum_distance_penalty);
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion = true;
  if (!node->has_parameter("use_response_expansion")) {
    node->declare_parameter("use_response_expansion", use_response_expansion);
  }
  node->get_parameter("use_response_expansion", use_response_expansion);
  mapper_->setParamUseResponseExpansion(use_response_expansion);


  int min_pass_through = 2;
  if (!node->has_parameter("min_pass_through")) {
    node->declare_parameter("min_pass_through", min_pass_through);
  }
  node->get_parameter("min_pass_through", min_pass_through);
  mapper_->setParamMinPassThrough(min_pass_through);

  double occupancy_threshold = 0.1;
  if (!node->has_parameter("occupancy_threshold")) {
    node->declare_parameter("occupancy_threshold", occupancy_threshold);
  }
  node->get_parameter("occupancy_threshold", occupancy_threshold);
  mapper_->setParamOccupancyThreshold(occupancy_threshold);

  // Reflector-aided sequential matching (REFLECTOR_SLAM.md §5). use_reflectors is also read by
  // SlamToolbox, which attaches the scans' reflector weights.
  bool use_reflectors = false;
  if (!node->has_parameter("use_reflectors")) {
    node->declare_parameter("use_reflectors", use_reflectors);
  }
  node->get_parameter("use_reflectors", use_reflectors);
  mapper_->setParamUseReflectors(use_reflectors);

  bool use_geometry = true;
  if (!node->has_parameter("use_geometry")) {
    node->declare_parameter("use_geometry", use_geometry);
  }
  node->get_parameter("use_geometry", use_geometry);
  if (!use_geometry && !use_reflectors) {
    RCLCPP_WARN(node->get_logger(),
      "use_geometry is false but use_reflectors is false too; matching on geometry.");
    use_geometry = true;
  }
  if (!use_geometry) {
    RCLCPP_WARN(node->get_logger(),
      "use_geometry is false: matching on reflectors only; loop closure has nothing to match "
      "unless use_reflectors_in_loop_closure is true.");
  }
  mapper_->setParamUseGeometry(use_geometry);

  double reflector_weight = 1.0;
  if (!node->has_parameter("reflector_weight")) {
    node->declare_parameter("reflector_weight", reflector_weight);
  }
  node->get_parameter("reflector_weight", reflector_weight);
  mapper_->setParamReflectorWeight(reflector_weight);

  double reflector_smear_deviation = 0.02;
  if (!node->has_parameter("reflector_smear_deviation")) {
    node->declare_parameter("reflector_smear_deviation", reflector_smear_deviation);
  }
  node->get_parameter("reflector_smear_deviation", reflector_smear_deviation);
  mapper_->setParamReflectorSmearDeviation(reflector_smear_deviation);

  double reflector_response_cap = 3.0;
  if (!node->has_parameter("reflector_response_cap")) {
    node->declare_parameter("reflector_response_cap", reflector_response_cap);
  }
  node->get_parameter("reflector_response_cap", reflector_response_cap);
  mapper_->setParamReflectorResponseCap(reflector_response_cap);

  // score: add the two responses (stage 3); information: fuse separate geometry and reflector
  // estimates by their covariances, with a consistency gate (REFLECTOR_SLAM.md §5.5).
  std::string reflector_fusion = "score";
  if (!node->has_parameter("reflector_fusion")) {
    node->declare_parameter("reflector_fusion", reflector_fusion);
  }
  node->get_parameter("reflector_fusion", reflector_fusion);
  if (reflector_fusion != "score" && reflector_fusion != "information") {
    RCLCPP_WARN(node->get_logger(),
      "Unknown reflector_fusion '%s' (score or information); using score.",
      reflector_fusion.c_str());
    reflector_fusion = "score";
  }
  mapper_->setParamReflectorInformationFusion(reflector_fusion == "information");

  double reflector_covariance_window = 0.9;
  if (!node->has_parameter("reflector_covariance_window")) {
    node->declare_parameter("reflector_covariance_window", reflector_covariance_window);
  }
  node->get_parameter("reflector_covariance_window", reflector_covariance_window);
  mapper_->setParamReflectorCovarianceWindow(reflector_covariance_window);

  double reflector_consistency_gate_chi2 = 11.34;
  if (!node->has_parameter("reflector_consistency_gate_chi2")) {
    node->declare_parameter("reflector_consistency_gate_chi2", reflector_consistency_gate_chi2);
  }
  node->get_parameter("reflector_consistency_gate_chi2", reflector_consistency_gate_chi2);
  mapper_->setParamReflectorConsistencyGateChi2(reflector_consistency_gate_chi2);

  bool reflector_fuse_heading = true;
  if (!node->has_parameter("reflector_fuse_heading")) {
    node->declare_parameter("reflector_fuse_heading", reflector_fuse_heading);
  }
  node->get_parameter("reflector_fuse_heading", reflector_fuse_heading);
  mapper_->setParamReflectorFuseHeading(reflector_fuse_heading);

  bool use_reflectors_in_loop_closure = false;
  if (!node->has_parameter("use_reflectors_in_loop_closure")) {
    node->declare_parameter("use_reflectors_in_loop_closure", use_reflectors_in_loop_closure);
  }
  node->get_parameter("use_reflectors_in_loop_closure", use_reflectors_in_loop_closure);
  mapper_->setParamReflectorsInLoopClosure(use_reflectors_in_loop_closure);

  // Loop closure in a featureless corridor (REFLECTOR_SLAM.md §5.9).
  bool loop_match_relative_covariance = false;
  if (!node->has_parameter("loop_match_relative_covariance")) {
    node->declare_parameter("loop_match_relative_covariance", loop_match_relative_covariance);
  }
  node->get_parameter("loop_match_relative_covariance", loop_match_relative_covariance);
  mapper_->setParamLoopMatchRelativeCovariance(loop_match_relative_covariance);

  bool loop_closure_coarse_covariance = false;
  if (!node->has_parameter("loop_closure_coarse_covariance")) {
    node->declare_parameter("loop_closure_coarse_covariance", loop_closure_coarse_covariance);
  }
  node->get_parameter("loop_closure_coarse_covariance", loop_closure_coarse_covariance);
  mapper_->setParamLoopClosureCoarseCovariance(loop_closure_coarse_covariance);

  // Stage 6 (REFLECTOR_SLAM.md 5.10): odometry prior.
  bool odometry_prior_fusion = false;
  if (!node->has_parameter("odometry_prior_fusion")) {
    node->declare_parameter("odometry_prior_fusion", odometry_prior_fusion);
  }
  node->get_parameter("odometry_prior_fusion", odometry_prior_fusion);
  mapper_->setParamOdometryPriorFusion(odometry_prior_fusion);

  double odometry_prior_stddev_xy_per_m = 0.05;
  if (!node->has_parameter("odometry_prior_stddev_xy_per_m")) {
    node->declare_parameter("odometry_prior_stddev_xy_per_m", odometry_prior_stddev_xy_per_m);
  }
  node->get_parameter("odometry_prior_stddev_xy_per_m", odometry_prior_stddev_xy_per_m);
  mapper_->setParamOdometryPriorStddevXyPerMeter(odometry_prior_stddev_xy_per_m);

  double odometry_prior_stddev_xy_min = 0.005;
  if (!node->has_parameter("odometry_prior_stddev_xy_min")) {
    node->declare_parameter("odometry_prior_stddev_xy_min", odometry_prior_stddev_xy_min);
  }
  node->get_parameter("odometry_prior_stddev_xy_min", odometry_prior_stddev_xy_min);
  mapper_->setParamOdometryPriorStddevXyMin(odometry_prior_stddev_xy_min);

  double odometry_prior_stddev_yaw_per_rad = 0.05;
  if (!node->has_parameter("odometry_prior_stddev_yaw_per_rad")) {
    node->declare_parameter("odometry_prior_stddev_yaw_per_rad", odometry_prior_stddev_yaw_per_rad);
  }
  node->get_parameter("odometry_prior_stddev_yaw_per_rad", odometry_prior_stddev_yaw_per_rad);
  mapper_->setParamOdometryPriorStddevYawPerRadian(odometry_prior_stddev_yaw_per_rad);

  double odometry_prior_stddev_yaw_per_m = 0.01;
  if (!node->has_parameter("odometry_prior_stddev_yaw_per_m")) {
    node->declare_parameter("odometry_prior_stddev_yaw_per_m", odometry_prior_stddev_yaw_per_m);
  }
  node->get_parameter("odometry_prior_stddev_yaw_per_m", odometry_prior_stddev_yaw_per_m);
  mapper_->setParamOdometryPriorStddevYawPerMeter(odometry_prior_stddev_yaw_per_m);

  double odometry_prior_stddev_yaw_min = 0.002;
  if (!node->has_parameter("odometry_prior_stddev_yaw_min")) {
    node->declare_parameter("odometry_prior_stddev_yaw_min", odometry_prior_stddev_yaw_min);
  }
  node->get_parameter("odometry_prior_stddev_yaw_min", odometry_prior_stddev_yaw_min);
  mapper_->setParamOdometryPriorStddevYawMin(odometry_prior_stddev_yaw_min);

  double odometry_prior_gate_chi2 = 11.34;
  if (!node->has_parameter("odometry_prior_gate_chi2")) {
    node->declare_parameter("odometry_prior_gate_chi2", odometry_prior_gate_chi2);
  }
  node->get_parameter("odometry_prior_gate_chi2", odometry_prior_gate_chi2);
  mapper_->setParamOdometryPriorGateChi2(odometry_prior_gate_chi2);

  double odometry_prior_stddev_cross_per_m = 0.05;
  if (!node->has_parameter("odometry_prior_stddev_cross_per_m")) {
    node->declare_parameter("odometry_prior_stddev_cross_per_m",
      odometry_prior_stddev_cross_per_m);
  }
  node->get_parameter("odometry_prior_stddev_cross_per_m", odometry_prior_stddev_cross_per_m);
  mapper_->setParamOdometryPriorStddevCrossPerMeter(odometry_prior_stddev_cross_per_m);

  bool odometry_prior_fuse_heading = false;
  if (!node->has_parameter("odometry_prior_fuse_heading")) {
    node->declare_parameter("odometry_prior_fuse_heading", odometry_prior_fuse_heading);
  }
  node->get_parameter("odometry_prior_fuse_heading", odometry_prior_fuse_heading);
  mapper_->setParamOdometryPriorFuseHeading(odometry_prior_fuse_heading);

  double near_chain_minimum_response_coarse = 0.35;
  if (!node->has_parameter("near_chain_minimum_response_coarse")) {
    node->declare_parameter("near_chain_minimum_response_coarse",
      near_chain_minimum_response_coarse);
  }
  node->get_parameter("near_chain_minimum_response_coarse", near_chain_minimum_response_coarse);
  mapper_->setParamNearChainMinimumResponseCoarse(near_chain_minimum_response_coarse);

  int link_chain_scans = 1;
  if (!node->has_parameter("link_chain_scans")) {
    node->declare_parameter("link_chain_scans", link_chain_scans);
  }
  node->get_parameter("link_chain_scans", link_chain_scans);
  mapper_->setParamLinkChainScans(link_chain_scans);

  bool near_chain_gated = false;
  if (!node->has_parameter("near_chain_gated")) {
    node->declare_parameter("near_chain_gated", near_chain_gated);
  }
  node->get_parameter("near_chain_gated", near_chain_gated);
  mapper_->setParamNearChainGated(near_chain_gated);

  bool loop_closure_move_scan = true;
  if (!node->has_parameter("loop_closure_move_scan")) {
    node->declare_parameter("loop_closure_move_scan", loop_closure_move_scan);
  }
  node->get_parameter("loop_closure_move_scan", loop_closure_move_scan);
  mapper_->setParamLoopClosureMoveScan(loop_closure_move_scan);

  double loop_closure_gate_chi2 = 0.0;
  if (!node->has_parameter("loop_closure_gate_chi2")) {
    node->declare_parameter("loop_closure_gate_chi2", loop_closure_gate_chi2);
  }
  node->get_parameter("loop_closure_gate_chi2", loop_closure_gate_chi2);
  mapper_->setParamLoopClosureGateChi2(loop_closure_gate_chi2);

  double loop_closure_gate_drift_xy = 0.05;
  if (!node->has_parameter("loop_closure_gate_drift_xy")) {
    node->declare_parameter("loop_closure_gate_drift_xy", loop_closure_gate_drift_xy);
  }
  node->get_parameter("loop_closure_gate_drift_xy", loop_closure_gate_drift_xy);
  mapper_->setParamLoopClosureGateDriftXY(loop_closure_gate_drift_xy);

  double loop_closure_gate_drift_yaw = 0.01;
  if (!node->has_parameter("loop_closure_gate_drift_yaw")) {
    node->declare_parameter("loop_closure_gate_drift_yaw", loop_closure_gate_drift_yaw);
  }
  node->get_parameter("loop_closure_gate_drift_yaw", loop_closure_gate_drift_yaw);
  mapper_->setParamLoopClosureGateDriftYaw(loop_closure_gate_drift_yaw);
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
}

}  // namespace mapper_utils
