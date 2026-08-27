/*
 * loop_closure_assistant
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

#include <unordered_map>
#include <memory>
#include <cmath>

#include "slam_toolbox/loop_closure_assistant.hpp"

namespace loop_closure_assistant
{

/*****************************************************************************/
LoopClosureAssistant::LoopClosureAssistant(
  rclcpp::Node::SharedPtr node,
  karto::Mapper * mapper,
  laser_utils::ScanHolder * scan_holder,
  PausedState & state, ProcessType & processor_type, tf2_ros::Buffer * tf)
: mapper_(mapper), scan_holder_(scan_holder),
  interactive_mode_(false), node_(node), state_(state),
  processor_type_(processor_type), tf_(tf)
/*****************************************************************************/
{
  node_->declare_parameter("paused_processing", false);
  node_->set_parameter(rclcpp::Parameter("paused_processing", false));
  node_->declare_parameter("interactive_mode", false);
  node_->set_parameter(rclcpp::Parameter("interactive_mode", false));
  node_->get_parameter("enable_interactive_mode", enable_interactive_mode_);

  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  solver_ = mapper_->getScanSolver();

  ssClear_manual_ = node_->create_service<slam_toolbox::srv::Clear>(
    "slam_toolbox/clear_changes", std::bind(&LoopClosureAssistant::clearChangesCallback, 
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  ssLoopClosure_ = node_->create_service<slam_toolbox::srv::LoopClosure>(
    "slam_toolbox/manual_loop_closure", std::bind(&LoopClosureAssistant::manualLoopClosureCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "slam_toolbox/scan_visualization",10);
  interactive_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "slam_toolbox",
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_services_interface());
  ssInteractive_ = node_->create_service<slam_toolbox::srv::ToggleInteractive>(
    "slam_toolbox/toggle_interactive_mode", std::bind(&LoopClosureAssistant::interactiveModeCallback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "slam_toolbox/graph_visualization", rclcpp::QoS(1));
  map_frame_ = node->get_parameter("map_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();

  // Debug visualization of the live, jointly-optimized sensor extrinsic estimate (see
  // PLAN.md step 10). Empty/unpublished unless optimize_sensor_extrinsics is enabled -
  // solver_->GetSensorOffsetCorrections() returns an empty map otherwise.
  extrinsic_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "slam_toolbox/sensor_extrinsic_calibration", rclcpp::QoS(1));

  // Frame to publish the extrinsic debug pose in. Left unset, falls back to base_frame_ (the
  // original behavior). Set to the lidar's own TF frame (e.g. "lidar_link") to instead publish
  // the residual between the live estimate and its frozen nominal, landing the marker where a
  // deliberately-offset link (e.g. a Gazebo test rig's lidar_offset_link) would be -- directly
  // comparable in RViz.
  node_->declare_parameter("lidar_frame", std::string(""));
  lidar_frame_ = node_->get_parameter("lidar_frame").as_string();
  if (lidar_frame_.empty()) {
    lidar_frame_ = base_frame_;
  }

  // Republish the last computed extrinsic estimate at a fixed, faster rate than publishGraph()
  // runs at (map_update_interval / loop closures / periodic solve), purely so RViz's
  // PoseWithCovariance display keeps redrawing smoothly between real updates. No-op when
  // nothing has been computed yet (e.g. extrinsic calibration disabled).
  node_->declare_parameter("extrinsic_publish_rate", 10.0);
  const double extrinsic_publish_rate = node_->get_parameter("extrinsic_publish_rate").as_double();
  extrinsic_publish_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / extrinsic_publish_rate),
    std::bind(&LoopClosureAssistant::publishExtrinsicCalibration, this));
}

/*****************************************************************************/
void LoopClosureAssistant::setMapper(karto::Mapper * mapper)
/*****************************************************************************/
{
  mapper_ = mapper;
}

/*****************************************************************************/
void LoopClosureAssistant::processInteractiveFeedback(const
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5, 
      "Interactive mode is invalid outside processing mode.");
    return;
  }

  const int id = std::stoi(feedback->marker_name, nullptr, 10);

  // was depressed, something moved, and now released
  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP &&
      feedback->mouse_point_valid)
  {
    addMovedNodes(id, Eigen::Vector3d(feedback->mouse_point.x,
      feedback->mouse_point.y, tf2::getYaw(feedback->pose.orientation)));
  }

  // is currently depressed, being moved before release
  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    // get scan
    sensor_msgs::msg::LaserScan scan = scan_holder_->getCorrectedScan(id);

    // get correct orientation
    tf2::Quaternion quat(0.,0.,0.,1.0), msg_quat(0.,0.,0.,1.0);
    double node_yaw, first_node_yaw;
    solver_->GetNodeOrientation(id, node_yaw);
    solver_->GetNodeOrientation(0, first_node_yaw);
    tf2::Quaternion q1(0.,0.,0.,1.0);
    q1.setEuler(0., 0., node_yaw - 3.14159);
    tf2::Quaternion q2(0.,0.,0.,1.0);
    q2.setEuler(0., 0., 3.14159);
    quat *= q1;
    quat *= q2;

    // interactive move
    tf2::convert(feedback->pose.orientation, msg_quat);
    quat *= msg_quat;
    quat.normalize();

    // create correct transform
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(feedback->pose.position.x,
      feedback->pose.position.y, 0.));
    transform.setRotation(quat);

    // publish the scan visualization with transform
    geometry_msgs::msg::TransformStamped msg;
    tf2::convert(transform, msg.transform);
    msg.child_frame_id = "scan_visualization";
    msg.header.frame_id = feedback->header.frame_id;
    msg.header.stamp = node_->now();
    tfB_->sendTransform(msg);

    scan.header.frame_id = "scan_visualization";
    scan.header.stamp = node_->now();
    scan_publisher_->publish(scan);
  }
}

/*****************************************************************************/
void LoopClosureAssistant::publishGraph()
/*****************************************************************************/
{
  interactive_server_->clear();
  auto graph = solver_->getGraph();

  if (graph->size() == 0) {
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Graph size: %zu", graph->size());
  bool interactive_mode = false;
  {
    boost::mutex::scoped_lock lock(interactive_mutex_);
    interactive_mode = interactive_mode_;
  }

  const auto & vertices = mapper_->GetGraph()->GetVertices();
  const auto & edges = mapper_->GetGraph()->GetEdges();
  const auto & localization_vertices = mapper_->GetLocalizationVertices();

  int first_localization_id = std::numeric_limits<int>::max();
  if (!localization_vertices.empty()) {
    first_localization_id = localization_vertices.front().vertex->GetObject()->GetUniqueId();
  }

  visualization_msgs::msg::MarkerArray marray;

  // clear existing markers to account for any removed nodes
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = node_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marray.markers.push_back(clear);

  visualization_msgs::msg::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, node_);

  // add map nodes
  for (const auto & sensor_name : vertices) {
    for (const auto & vertex : sensor_name.second) {
      m.color.g = vertex.first < first_localization_id ? 0.0 : 1.0;
      const auto & pose = vertex.second->GetObject()->GetCorrectedPose();
      m.id = vertex.first;
      m.pose.position.x = pose.GetX();
      m.pose.position.y = pose.GetY();

      if (interactive_mode && enable_interactive_mode_) {
        visualization_msgs::msg::InteractiveMarker int_marker =
          vis_utils::toInteractiveMarker(m, 0.3, node_);
        interactive_server_->insert(int_marker,
          std::bind(
          &LoopClosureAssistant::processInteractiveFeedback,
          this, std::placeholders::_1));
      } else {
        marray.markers.push_back(m);
      }
    }
  }

  // add line markers for graph edges
  visualization_msgs::msg::Marker edges_marker;
  edges_marker.header.frame_id = map_frame_;
  edges_marker.header.stamp = node_->now();
  edges_marker.id = 0;
  edges_marker.ns = "slam_toolbox_edges";
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.pose.orientation.w = 1;
  edges_marker.scale.x = 0.05;
  edges_marker.color.b = 1;
  edges_marker.color.a = 1;
  edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  edges_marker.points.reserve(edges.size() * 2);

  visualization_msgs::msg::Marker localization_edges_marker;
  localization_edges_marker.header.frame_id = map_frame_;
  localization_edges_marker.header.stamp = node_->now();
  localization_edges_marker.id = 1;
  localization_edges_marker.ns = "slam_toolbox_edges";
  localization_edges_marker.action = visualization_msgs::msg::Marker::ADD;
  localization_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  localization_edges_marker.pose.orientation.w = 1;
  localization_edges_marker.scale.x = 0.05;
  localization_edges_marker.color.g = 1;
  localization_edges_marker.color.b = 1;
  localization_edges_marker.color.a = 1;
  localization_edges_marker.lifetime = rclcpp::Duration::from_seconds(0);
  localization_edges_marker.points.reserve(localization_vertices.size() * 3);

  for (const auto & edge : edges) {
    int source_id = edge->GetSource()->GetObject()->GetUniqueId();
    const auto & pose0 = edge->GetSource()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p0;
    p0.x = pose0.GetX();
    p0.y = pose0.GetY();

    int target_id = edge->GetTarget()->GetObject()->GetUniqueId();
    const auto & pose1 = edge->GetTarget()->GetObject()->GetCorrectedPose();
    geometry_msgs::msg::Point p1;
    p1.x = pose1.GetX();
    p1.y = pose1.GetY();

    if (source_id >= first_localization_id || target_id >= first_localization_id) {
      localization_edges_marker.points.push_back(p0);
      localization_edges_marker.points.push_back(p1);
    } else {
      edges_marker.points.push_back(p0);
      edges_marker.points.push_back(p1);
    }
  }

  marray.markers.push_back(edges_marker);
  marray.markers.push_back(localization_edges_marker);

  // if disabled, clears out old markers
  interactive_server_->applyChanges();
  marker_publisher_->publish(marray);

  // compute/cache the live sensor extrinsic (lidar<->base_link) calibration estimate, if any -
  // empty/no-op unless optimize_sensor_extrinsics is enabled and at least one solve has
  // completed. Published relative to its frozen nominal, in lidar_frame_, so it's directly
  // comparable to e.g. a test rig's deliberately-offset lidar link in RViz.
  const auto & offset_corrections = solver_->GetSensorOffsetCorrections();
  const auto & offset_covariances = solver_->GetSensorOffsetCovariances();
  const auto & nominal_offsets = solver_->GetSensorNominalOffsets();
  for (const auto & sensorCorrection : offset_corrections) {
    const karto::Pose2 & estimate = sensorCorrection.second;

    // residual = nominal^-1 . estimate, i.e. "estimate expressed in the nominal frame" -- same
    // Transform-based composition LinkInfo::Update/UpdateSensorFrame already use.
    karto::Pose2 nominal;
    const auto nominal_it = nominal_offsets.find(sensorCorrection.first);
    if (nominal_it != nominal_offsets.end()) {
      nominal = nominal_it->second;
    }
    karto::Transform toNominalFrame(nominal, karto::Pose2());
    const karto::Pose2 residual = toNominalFrame.TransformPose(estimate);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = lidar_frame_;
    pose_msg.header.stamp = node_->now();

    // lidar_frame_ may be mounted upside-down relative to base_frame_ (Z axis flipped in
    // world space). karto/Ceres only ever reasons in 2D, so every yaw value it produces is
    // implicitly "measured about base_frame_'s own upright Z axis" -- publishing that same
    // value as a rotation about a Z-flipped child frame's own axis silently negates its visual
    // sense (R.Rz(th).R^-1 = Rz(-th) whenever R maps Z -> -Z). Correct for it here.
    double published_heading = residual.GetHeading();
    if (isLidarFrameInverted()) {
      published_heading = -published_heading;
    }

    tf2::Quaternion q(0., 0., 0., 1.0);
    q.setRPY(0., 0., published_heading);
    tf2::Transform transform(q, tf2::Vector3(residual.GetX(), residual.GetY(), 0.0));
    tf2::toMsg(transform, pose_msg.pose.pose);

    const auto cov_it = offset_covariances.find(sensorCorrection.first);
    if (cov_it != offset_covariances.end()) {
      // rotate the covariance into the same (nominal) frame the position was just expressed in
      Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      const double c = std::cos(-nominal.GetHeading());
      const double s = std::sin(-nominal.GetHeading());
      rot(0, 0) = c; rot(0, 1) = -s;
      rot(1, 0) = s; rot(1, 1) = c;
      const Eigen::Matrix3d cov = rot * cov_it->second * rot.transpose();
      pose_msg.pose.covariance[0] = cov(0, 0);   // x
      pose_msg.pose.covariance[1] = cov(0, 1);   // xy
      pose_msg.pose.covariance[6] = cov(1, 0);   // xy
      pose_msg.pose.covariance[7] = cov(1, 1);   // y
      pose_msg.pose.covariance[35] = cov(2, 2);  // yaw
    }

    boost::mutex::scoped_lock lock(extrinsic_mutex_);
    last_extrinsic_msgs_[sensorCorrection.first] = pose_msg;
  }

  publishExtrinsicCalibration();
}

/*****************************************************************************/
void LoopClosureAssistant::publishExtrinsicCalibration()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(extrinsic_mutex_);
  const rclcpp::Time now = node_->now();
  for (auto & kv : last_extrinsic_msgs_) {
    kv.second.header.stamp = now;
    extrinsic_pub_->publish(kv.second);
  }
}

/*****************************************************************************/
bool LoopClosureAssistant::isLidarFrameInverted()
/*****************************************************************************/
{
  if (lidar_frame_inversion_checked_) {
    return lidar_frame_inverted_;
  }

  // Same technique laser_utils::LaserAssistant::isInverted() uses: rotate the "up" vector from
  // base_frame_ into lidar_frame_ and see if it comes out pointing down. A no-op (never
  // inverted) when lidar_frame_ == base_frame_, e.g. the lidar_frame param was left unset.
  geometry_msgs::msg::Vector3Stamped up_in_base;
  up_in_base.header.frame_id = base_frame_;
  up_in_base.vector.x = 0.0;
  up_in_base.vector.y = 0.0;
  up_in_base.vector.z = 1.0;

  try {
    geometry_msgs::msg::Vector3Stamped up_in_lidar = tf_->transform(up_in_base, lidar_frame_);
    lidar_frame_inverted_ = (up_in_lidar.vector.z <= 0.0);
    lidar_frame_inversion_checked_ = true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG(node_->get_logger(),
      "isLidarFrameInverted: TF not yet available (%s), will retry.", e.what());
  }

  return lidar_frame_inverted_;
}

/*****************************************************************************/
bool LoopClosureAssistant::manualLoopClosureCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::LoopClosure::Request> req, 
  std::shared_ptr<slam_toolbox::srv::LoopClosure::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      node_->get_logger(), "Called manual loop closure"
      " with interactive mode disabled. Ignoring.");
    return false;
  }

  {
    boost::mutex::scoped_lock lock(moved_nodes_mutex_);

    if (moved_nodes_.size() == 0)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "No moved nodes to attempt manual loop closure.");
      return true;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "LoopClosureAssistant: Attempting to manual "
      "loop close with %i moved nodes.", (int)moved_nodes_.size());
    // for each in node map
    std::map<int, Eigen::Vector3d>::const_iterator it = moved_nodes_.begin();
    for (it; it != moved_nodes_.end(); ++it)
    {
      moveNode(it->first,
        Eigen::Vector3d(it->second(0),it->second(1), it->second(2)));
    }
  }

  // optimize
  mapper_->CorrectPoses();

  //update visualization and clear out nodes completed
  publishGraph();
  clearMovedNodes();
  return true;
}


/*****************************************************************************/
bool LoopClosureAssistant::interactiveModeCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::ToggleInteractive::Request>  req,
  std::shared_ptr<slam_toolbox::srv::ToggleInteractive::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Called toggle interactive mode with interactive mode disabled. Ignoring.");
    return false;
  }

  bool interactive_mode;
  {
    boost::mutex::scoped_lock lock_i(interactive_mutex_);
    interactive_mode_ = !interactive_mode_;
    interactive_mode = interactive_mode_;
    node_->set_parameter(rclcpp::Parameter("interactive_mode", interactive_mode_));
  }

  RCLCPP_INFO(node_->get_logger(),
     "SlamToolbox: Toggling %s interactive mode.",
      interactive_mode ? "on" : "off");
  publishGraph();
  clearMovedNodes();

  // set state so we don't overwrite changes in rviz while loop closing
  state_.set(PROCESSING, interactive_mode);
  state_.set(VISUALIZING_GRAPH, interactive_mode);
  node_->set_parameter(rclcpp::Parameter("paused_processing", interactive_mode));
  return true;
}

/*****************************************************************************/
void LoopClosureAssistant::moveNode(
  const int & id, const Eigen::Vector3d & pose)
/*****************************************************************************/
{
  solver_->ModifyNode(id, pose);
}

/*****************************************************************************/
bool LoopClosureAssistant::clearChangesCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::Clear::Request> req, 
  std::shared_ptr<slam_toolbox::srv::Clear::Response> resp)
/*****************************************************************************/
{
  if(!enable_interactive_mode_)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Called Clear changes with interactive mode disabled. Ignoring.");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "LoopClosureAssistant: Clearing manual loop closure nodes.");
  publishGraph();
  clearMovedNodes();
  return true;
}

/*****************************************************************************/
void  LoopClosureAssistant::clearMovedNodes()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_.clear();
}

/*****************************************************************************/
void LoopClosureAssistant::addMovedNodes(const int & id, Eigen::Vector3d vec)
/*****************************************************************************/
{
  RCLCPP_INFO(
    node_->get_logger(),
    "LoopClosureAssistant: Node %i new manual loop closure "
    "pose has been recorded.",id);
  boost::mutex::scoped_lock lock(moved_nodes_mutex_);
  moved_nodes_[id] = vec;
}

}  // namespace loop_closure_assistant
