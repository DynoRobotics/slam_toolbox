// Deterministic offline replay: feeds a recorded bag through slam_toolbox's mapping pipeline
// without ROS timing, so runs with the same input and parameters give identical poses
// (REFLECTOR_SLAM.md §7).
//
// Usage:
//   offline_replay <bag> <out_prefix> [--save] [--surface <file>] [--loop-surface <file>]
//     [--viz] [--rate <factor>] [--start <seconds>] --ros-args --params-file <slam params> [-p ...]
//
// The bag needs the scan topic (scan_topic, e.g. /scan_filtered) and /tf, /tf_static.
// /slam_toolbox/pause, when present (scan_offline writes scan_node's), pauses new
// measurements in bag order, as the live node's subscription does. Messages
// are read in bag order; /tf goes straight into the transform buffer, and each scan waits until
// odom has been read past its stamp, then goes through the same steps as the synchronous node
// (odom pose, shouldProcessScan, addScan), one at a time. Writes <out_prefix>_poses.csv with the
// pose of every processed scan when it was added (online) and after the last scan (final), what
// the reflector layer did in its sequential match, and
// with --save also <out_prefix>.posegraph/.data. --surface writes every candidate pose of the
// sequential matcher's searches to <file> (REFLECTOR_SLAM.md 5.10; scripts/reflector_eval/
// surface.py reads it). --loop-surface writes each coarse search of the loop matcher (loop
// closures and gated near chains) as a grid of the best response per position
// (ScanMatcher::SetPositionSurfaceDump). Every gated match attempt goes to
// <out_prefix>_gated.csv: search centre, coarse mean and covariance before any ridge
// inflation, fine mean, outcome, and the chain scan it links to. With stationary_correction,
// every correction attempt goes to <out_prefix>_corrections.csv: predicted and matched robot
// pose, response, the gate's verdict and heading limit.
//
// For watching in RViz: the node publishes /map, /slam_toolbox/graph_visualization and /pose as
// the live one does, the map and graph every map_update_interval of wall time (the map only
// while someone subscribes). --rate <factor> paces the replay to at most factor times real
// time (scan stamps), so the map can be seen growing; without it the replay runs as fast as it
// can. --viz keeps the node up after the last scan, publishing the final map and graph, until
// Ctrl-C. Ctrl-C during the replay stops reading and writes the outputs for the scans so far.
// Neither changes the results.
//
// --start <seconds> skips the scans in the first <seconds> after the bag's first scan (by scan
// stamp); slam starts on the first scan after that, with the transforms before it already in
// the buffer.

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/serialization.hpp"
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace slam_toolbox
{

class OfflineReplay : public SlamToolbox
{
public:
  explicit OfflineReplay(rclcpp::NodeOptions options)
  : SlamToolbox(options) {}

  int run(
    const std::string & bag, const std::string & out_prefix, bool save, double rate,
    double start_offset)
  {
    const auto start = std::chrono::steady_clock::now();
    rosbag2_storage::StorageOptions storage;
    storage.uri = bag;
    storage.storage_id = hasMcap(bag) ? "mcap" : "sqlite3";
    rosbag2_cpp::Reader reader;
    reader.open(storage);

    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
    // Scans waiting for odom, and pause changes kept in order with them: a pause takes effect
    // once every scan read before it has gone.
    std::deque<std::variant<sensor_msgs::msg::LaserScan::ConstSharedPtr, bool>> pending;
    rclcpp::Serialization<std_msgs::msg::Bool> pause_serialization;
    rclcpp::Time odom_until(0, 0, RCL_ROS_TIME);
    // --rate: the first scan's stamp and the wall time it was read.
    std::optional<std::pair<rclcpp::Time, std::chrono::steady_clock::time_point>> pace_start;
    // --start: scans stamped before this are skipped (set from the first scan).
    std::optional<rclcpp::Time> start_at;
    std::size_t skipped = 0;

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "offline_replay: interrupted, writing what was replayed");
        break;
      }
      const auto bag_msg = reader.read_next();
      const rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      if (bag_msg->topic_name == "/tf" || bag_msg->topic_name == "/tf_static") {
        const bool is_static = bag_msg->topic_name == "/tf_static";
        tf2_msgs::msg::TFMessage tf_msg;
        tf_serialization.deserialize_message(&serialized, &tf_msg);
        for (const auto & t : tf_msg.transforms) {
          tf_->setTransform(t, "bag", is_static);
          // Any odom-parented transform: the odom -> base chain may go through other frames
          // (here odom -> root_link, with root_link -> base_link static).
          if (!is_static && t.header.frame_id == odom_frame_) {
            odom_until = std::max(odom_until, rclcpp::Time(t.header.stamp, RCL_ROS_TIME));
          }
        }
      } else if (bag_msg->topic_name == scan_topic_) {
        auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_serialization.deserialize_message(&serialized, scan.get());
        ++scans_read_;
        if (!start_at) {
          start_at = rclcpp::Time(scan->header.stamp, RCL_ROS_TIME) +
            rclcpp::Duration::from_seconds(start_offset);
        }
        if (rclcpp::Time(scan->header.stamp, RCL_ROS_TIME) < *start_at) {
          ++skipped;
          continue;
        }
        // Paced by scan stamps, not the bag's receive times, which can jump.
        if (rate > 0.0) {
          const rclcpp::Time stamp(scan->header.stamp, RCL_ROS_TIME);
          if (!pace_start) {
            pace_start.emplace(stamp, std::chrono::steady_clock::now());
          }
          const double ahead = (stamp - pace_start->first).seconds() / rate;
          if (ahead > 0.0) {
            std::this_thread::sleep_until(
              pace_start->second + std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(ahead)));
          }
        }
        pending.push_back(scan);
      } else if (bag_msg->topic_name == "slam_toolbox/pause" ||
        bag_msg->topic_name == "/slam_toolbox/pause")
      {
        std_msgs::msg::Bool pause;
        pause_serialization.deserialize_message(&serialized, &pause);
        pending.push_back(pause.data);
      } else {
        continue;
      }
      // Odom is interpolated, so a scan can go once odom has been read past its stamp.
      while (!pending.empty()) {
        if (const bool * pause = std::get_if<bool>(&pending.front())) {
          setPaused(*pause);
        } else {
          const auto & scan = std::get<sensor_msgs::msg::LaserScan::ConstSharedPtr>(pending.front());
          if (rclcpp::Time(scan->header.stamp, RCL_ROS_TIME) > odom_until) {
            break;
          }
          timedCallback(scan);
        }
        pending.pop_front();
      }
    }
    if (skipped > 0) {
      RCLCPP_INFO(get_logger(), "offline_replay: --start: skipped the first %zu scans (%.1f s)",
        skipped, start_offset);
    }
    for (const auto & item : pending) {
      if (!rclcpp::ok()) {
        break;
      }
      if (const bool * pause = std::get_if<bool>(&item)) {
        setPaused(*pause);
      } else {
        timedCallback(std::get<sensor_msgs::msg::LaserScan::ConstSharedPtr>(item));
      }
    }

    {
      std::ofstream f(out_prefix + "_timing.csv");
      f << "stamp,seconds,near,loop,match,solve,near_chains,near_chains_matched\n";
      for (const auto & c : callback_seconds_) {
        f << std::fixed << std::setprecision(3) << c.stamp << std::setprecision(4) << ','
          << c.seconds << ',' << c.near << ',' << c.loop << ',' << c.match << ',' << c.solve
          << ',' << c.nearChains << ',' << c.nearChainsMatched << '\n';
      }
    }
    writePoses(out_prefix + "_poses.csv");
    if (save) {
      serialization::write(out_prefix, *smapper_->getMapper(), *dataset_, shared_from_this());
    }
    const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    // Graph edges: sequential (consecutive scans) vs cross links (near chains, loop closures).
    std::size_t sequential = 0, cross = 0;
    for (const auto * edge : smapper_->getMapper()->GetGraph()->GetEdges()) {
      const int a = edge->GetSource()->GetObject()->GetUniqueId();
      const int b = edge->GetTarget()->GetObject()->GetUniqueId();
      (std::abs(a - b) <= 1 ? sequential : cross)++;
    }
    writeEdges(out_prefix + "_edges.csv");
    std::size_t corrections_accepted = 0;
    if (!corrections_.empty()) {
      FILE * f = std::fopen((out_prefix + "_corrections.csv").c_str(), "w");
      std::fprintf(f, "stamp_ns,accepted,reason,pred_x,pred_y,pred_yaw,match_x,match_y,match_yaw,"
        "response,heading_limit,position\n");
      for (const auto & [stamp, a] : corrections_) {
        corrections_accepted += a.verdict.accepted ? 1 : 0;
        std::fprintf(f, "%ld,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n", stamp,
          a.verdict.accepted ? 1 : 0, StationaryCorrection::toString(a.verdict.reason),
          a.predicted.GetX(), a.predicted.GetY(), a.predicted.GetHeading(), a.matched.GetX(),
          a.matched.GetY(), a.matched.GetHeading(), a.response, a.verdict.heading_limit,
          a.verdict.position ? 1 : 0);
      }
      std::fclose(f);
      RCLCPP_INFO(get_logger(), "offline_replay: stationary corrections: %zu attempts, %zu accepted",
        corrections_.size(), corrections_accepted);
    }
    writeGated(out_prefix + "_gated.csv");
    const auto & loops = smapper_->getMapper()->GetGraph()->GetLoopClosureStats();
    RCLCPP_INFO(get_logger(),
      "offline_replay: %zu scans read, %zu processed, %zu without odom, %zu paused, "
      "%u heading-gated, %.1f s, "
      "edges %zu sequential + %zu cross; loop closures: %u attempts, %u accepted (%u ridge, %u peak), "
      "rejected %u coarse response, %u variance, %u fine response, %u gate; reflectors in %u",
      scans_read_, online_.size(), no_odom_, paused_scans_,
      smapper_->getMapper()->GetSequentialHeadingRejections(), seconds, sequential, cross, loops.attempts,
      loops.accepted, loops.acceptedRidge, loops.acceptedPeak, loops.rejectedCoarseResponse, loops.rejectedVariance,
      loops.rejectedFineResponse, loops.rejectedGate, loops.reflectorsActive);
    logExtrinsics();
    logCoarseResponses("loop closure", loops.coarseResponses);
    const auto & near = smapper_->getMapper()->GetGraph()->GetNearChainStats();
    logCoarseResponses("gated near chain", near.coarseResponses);
    if (near.attempts > 0) {
      RCLCPP_INFO(get_logger(),
        "offline_replay: gated near chains: %u attempts, %u accepted (%u ridge, %u peak), rejected %u "
        "coarse response, %u variance, %u fine response, %u gate; reflectors in %u; not matched "
        "%u throttled, %u capped",
        near.attempts, near.accepted, near.acceptedRidge, near.acceptedPeak,
        near.rejectedCoarseResponse, near.rejectedVariance,
        near.rejectedFineResponse, near.rejectedGate, near.reflectorsActive,
        near.throttled, near.capped);
    }
    return 0;
  }

  // After the replay (--viz): the final map and graph now, then the visualization thread
  // republishes them every map_update_interval until Ctrl-C.
  void holdVisualization()
  {
    updateMap();
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      closure_assistant_->publishGraph();
    }
    RCLCPP_INFO(get_logger(), "offline_replay: --viz: showing the final map and graph, Ctrl-C to exit");
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

protected:
  // The synchronous node's laserCallback, with the queue replaced by an immediate addScan.
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override
  {
    karto::Pose2 pose;
    if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
      if (no_odom_++ == 0) {
        try {
          tf_->lookupTransform(odom_frame_, base_frame_, scan->header.stamp);
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN(get_logger(), "offline_replay: no odom pose: %s", e.what());
        }
      }
      return;
    }
    karto::LaserRangeFinder * laser = getLaser(scan);
    if (!laser) {
      return;
    }
    const karto::Pose2 sensor_pose = karto::Transform(pose).TransformPose(laser->GetOffsetPose());
    const ScanAction action = classifyScan(scan, sensor_pose, pose);
    if (action == ScanAction::CorrectOnly) {
      corrections_.push_back(
        {rclcpp::Time(scan->header.stamp).nanoseconds(), correctStationaryPose(laser, scan, pose)});
      return;
    }
    if (action != ScanAction::AddToGraph) {
      return;
    }
    if (surface_ && smapper_->getMapper()->GetSequentialScanMatcher()) {
      smapper_->getMapper()->GetSequentialScanMatcher()->SetSurfaceDump(surface_.get());
    }
    if (loop_surface_ && smapper_->getMapper()->GetGraph() &&
      smapper_->getMapper()->GetLoopScanMatcher())
    {
      smapper_->getMapper()->GetLoopScanMatcher()->SetPositionSurfaceDump(loop_surface_.get());
    }
    karto::LocalizedRangeScan * range_scan = addScan(laser, scan, pose);
    if (!range_scan) {
      undoScanAcceptance();
    }
    if (range_scan) {
      const karto::ScanMatcher * matcher = smapper_->getMapper()->GetSequentialScanMatcher();
      online_[range_scan->GetUniqueId()] =
        {rclcpp::Time(scan->header.stamp).nanoseconds(), range_scan->GetCorrectedPose(),
        matcher->GetLastReflectorStats(), matcher->GetLastMatch()};
    }
  }

private:
  struct Online
  {
    int64_t stamp_ns;
    karto::Pose2 pose;
    karto::ScanMatcher::ReflectorMatchStats reflectors;
    karto::ScanMatcher::MatchDiagnostics match;
  };

  // The online extrinsic calibration's final estimate per sensor (optimize_sensor_extrinsics)
  // against the nominal offset from TF.
  void logExtrinsics()
  {
    karto::ScanSolver * solver = smapper_->getMapper()->getScanSolver();
    if (solver == nullptr) {
      return;
    }
    const auto & nominal = solver->GetSensorNominalOffsets();
    for (const auto & [sensor, estimate] : solver->GetSensorOffsetCorrections()) {
      const auto it = nominal.find(sensor);
      if (it == nominal.end()) {
        continue;
      }
      RCLCPP_INFO(get_logger(),
        "offline_replay: extrinsic %s: estimate x %.4f y %.4f yaw %.3f deg; nominal x %.4f "
        "y %.4f yaw %.3f deg", sensor.c_str(), estimate.GetX(), estimate.GetY(),
        estimate.GetHeading() * 180.0 / M_PI, it->second.GetX(), it->second.GetY(),
        it->second.GetHeading() * 180.0 / M_PI);
      // As published and stored: the correction lidar_link -> lidar_link_calibrated, comparable
      // with a simulated mounting error (sim_lidar_error_*)
      const geometry_msgs::msg::Pose correction = closure_assistant_->toLidarFrame(
        karto::Transform(it->second).InverseTransformPose(estimate));
      RCLCPP_INFO(get_logger(),
        "offline_replay: extrinsic %s: correction in lidar frame x %.4f y %.4f yaw %.3f deg",
        sensor.c_str(), correction.position.x, correction.position.y,
        tf2::getYaw(correction.orientation) * 180.0 / M_PI);
      // its marginal standard deviations (robot frame), what afw_calibration's save gate reads
      const auto cov = solver->GetSensorOffsetCovariances().find(sensor);
      if (cov != solver->GetSensorOffsetCovariances().end()) {
        RCLCPP_INFO(get_logger(),
          "offline_replay: extrinsic %s: stddev x %.4f y %.4f yaw %.4f rad", sensor.c_str(),
          std::sqrt(cov->second(0, 0)), std::sqrt(cov->second(1, 1)),
          std::sqrt(cov->second(2, 2)));
      } else {
        RCLCPP_INFO(get_logger(), "offline_replay: extrinsic %s: no covariance", sensor.c_str());
      }
    }
  }

  // Percentiles of the coarse responses, for choosing the coarse thresholds.
  void logCoarseResponses(const char * what, std::vector<double> responses)
  {
    if (responses.empty()) {
      return;
    }
    std::sort(responses.begin(), responses.end());
    const auto at = [&](double q) {
        return responses[static_cast<std::size_t>(q * (responses.size() - 1))];
      };
    RCLCPP_INFO(get_logger(),
      "offline_replay: %s coarse responses: p10 %.3f, p50 %.3f, p90 %.3f, max %.3f (%zu)",
      what, at(0.1), at(0.5), at(0.9), responses.back(), responses.size());
  }

  static bool hasMcap(const std::string & bag)
  {
    if (std::filesystem::is_directory(bag)) {
      for (const auto & entry : std::filesystem::directory_iterator(bag)) {
        if (entry.path().extension() == ".mcap") {
          return true;
        }
      }
      return false;
    }
    return std::filesystem::path(bag).extension() == ".mcap";
  }

  void writePoses(const std::string & path)
  {
    FILE * f = std::fopen(path.c_str(), "w");
    std::fprintf(f, "id,stamp_ns,online_x,online_y,online_yaw,final_x,final_y,final_yaw,"
      "refl_active,refl_scan_points,refl_reference_points,refl_matched,response,"
      "refl_fused,refl_gated,refl_gate_d2,"
      "match,pred_x,pred_y,pred_yaw,scan_x,scan_y,scan_yaw,scan_cxx,scan_cxy,scan_cyy,scan_cxyaw,"
      "scan_cyyaw,scan_cyaw,prior,prior_gated,prior_d2\n");
    for (const karto::LocalizedRangeScan * scan : smapper_->getMapper()->GetAllProcessedScans()) {
      const auto it = online_.find(scan->GetUniqueId());
      if (it == online_.end()) {
        continue;
      }
      const karto::Pose2 & online = it->second.pose;
      const karto::Pose2 & final_pose = scan->GetCorrectedPose();
      const auto & r = it->second.reflectors;
      // Match diagnostics are of the sensor pose (what the matcher estimates).
      const auto & m = it->second.match;
      const karto::Matrix3 & c = m.scanCovariance;
      std::fprintf(f, "%d,%ld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%u,%u,%u,%.6f,%d,%d,%.3f,"
        "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%.3f\n",
        scan->GetUniqueId(), it->second.stamp_ns, online.GetX(), online.GetY(),
        online.GetHeading(), final_pose.GetX(), final_pose.GetY(), final_pose.GetHeading(),
        r.active ? 1 : 0, r.scanPoints, r.referencePoints, r.matched, r.response,
        r.fused ? 1 : 0, r.gated ? 1 : 0, r.gateD2, m.valid ? 1 : 0,
        m.predicted.GetX(), m.predicted.GetY(), m.predicted.GetHeading(), m.scanMean.GetX(),
        m.scanMean.GetY(), m.scanMean.GetHeading(), c(0, 0), c(0, 1), c(1, 1), c(0, 2), c(1, 2),
        c(2, 2), m.prior ? 1 : 0, m.priorGated ? 1 : 0, m.priorD2);
    }
    std::fclose(f);
  }

  // Every graph edge: the two scans, what created it, and the measured relative pose (robot
  // frame of the first scan) with its covariance, for scripts/reflector_eval/eval_edges.py.
  void writeGated(const std::string & path)
  {
    FILE * f = std::fopen(path.c_str(), "w");
    std::fprintf(f,
      "kind,stamp_ns,scan_id,chain_id,chain_stamp_ns,chain_x,chain_y,chain_yaw,centre_x,"
      "centre_y,centre_yaw,coarse_x,coarse_y,coarse_yaw,coarse_cxx,coarse_cxy,coarse_cyy,"
      "coarse_cyaw,coarse_response,variance_major,variance_minor,fine_x,fine_y,fine_yaw,"
      "fine_response,ridge,outcome,peak_tested,peaked,peak_contrast,runner_up_contrast,"
      "peak_offset,peak_sigma\n");
    const auto * graph = smapper_->getMapper()->GetGraph();
    const std::pair<const char *, const karto::MapperGraph::LoopClosureStats *> kinds[] = {
      {"loop", &graph->GetLoopClosureStats()}, {"near", &graph->GetNearChainStats()}};
    for (const auto & [kind, stats] : kinds) {
      for (const auto & a : stats->attemptLog) {
        const karto::Matrix3 & c = a.coarseCovariance;
        std::fprintf(f,
          "%s,%ld,%d,%d,%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g,%.6g,"
          "%.6f,%.6g,%.6g,%.6f,%.6f,%.6f,%.6f,%d,%u,%d,%d,%.5f,%.5f,%.4f,%.4f\n",
          kind, static_cast<int64_t>(a.time * 1e9), a.scanId, a.chainId,
          static_cast<int64_t>(a.chainTime * 1e9), a.chainPose.GetX(), a.chainPose.GetY(),
          a.chainPose.GetHeading(), a.centre.GetX(), a.centre.GetY(), a.centre.GetHeading(),
          a.coarseMean.GetX(), a.coarseMean.GetY(), a.coarseMean.GetHeading(), c(0, 0),
          c(0, 1), c(1, 1), c(2, 2), a.coarseResponse, a.varianceMajor, a.varianceMinor,
          a.fineMean.GetX(), a.fineMean.GetY(), a.fineMean.GetHeading(), a.fineResponse,
          a.ridge ? 1 : 0, a.outcome, a.peakTested ? 1 : 0, a.peaked ? 1 : 0, a.peakContrast,
          a.runnerUpContrast, a.peakOffset, a.peakSigma);
      }
    }
    std::fclose(f);
  }

  void writeEdges(const std::string & path)
  {
    FILE * f = std::fopen(path.c_str(), "w");
    std::fprintf(f, "from_id,to_id,from_stamp_ns,to_stamp_ns,origin,dx,dy,dyaw,cxx,cyy,cyaw\n");
    for (auto * edge : smapper_->getMapper()->GetGraph()->GetEdges()) {
      const karto::LocalizedRangeScan * from = edge->GetSource()->GetObject();
      const karto::LocalizedRangeScan * to = edge->GetTarget()->GetObject();
      auto * info = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
      if (info == nullptr) {
        continue;
      }
      const karto::Pose2 & d = info->GetPoseDifference();
      const karto::Matrix3 & c = info->GetCovariance();
      std::fprintf(f, "%d,%d,%ld,%ld,%u,%.6f,%.6f,%.6f,%.6g,%.6g,%.6g\n",
        from->GetUniqueId(), to->GetUniqueId(), static_cast<int64_t>(from->GetTime() * 1e9),
        static_cast<int64_t>(to->GetTime() * 1e9), info->GetOrigin(), d.GetX(), d.GetY(),
        d.GetHeading(), c(0, 0), c(1, 1), c(2, 2));
    }
    std::fclose(f);
  }

  std::map<int, Online> online_;
  // Stationary correction attempts (stationary_correction), with their scan stamps.
  std::vector<std::pair<int64_t, StationaryAttempt>> corrections_;

public:
  std::unique_ptr<std::ofstream> surface_;
  std::unique_ptr<std::ofstream> loop_surface_;

private:
  std::size_t scans_read_{0};
  // Wall time of each callback that added a scan (the live node's blocked-callback time).
  // With the time in each phase (karto::MapperGraph::PhaseSeconds) during that callback.
  struct CallbackTiming
  {
    double stamp, seconds, near, loop, match, solve;
    std::size_t nearChains, nearChainsMatched;
  };
  std::vector<CallbackTiming> callback_seconds_;

  // /slam_toolbox/pause from the bag (scan_node's), as the live node's subscription applies it.
  void setPaused(bool paused)
  {
    state_.set(NEW_MEASUREMENTS, paused);
  }

  void timedCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    if (isPaused(NEW_MEASUREMENTS)) {
      ++paused_scans_;
    }
    const std::size_t before = online_.size();
    const auto * graph = smapper_->getMapper()->GetGraph();
    const auto p0 = graph ? graph->GetPhaseSeconds() : karto::MapperGraph::PhaseSeconds{};
    const auto t0 = std::chrono::steady_clock::now();
    laserCallback(scan);
    if (online_.size() != before) {
      const auto & p1 = smapper_->getMapper()->GetGraph()->GetPhaseSeconds();
      callback_seconds_.push_back({rclcpp::Time(scan->header.stamp).seconds(),
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
          p1.near - p0.near, p1.loop - p0.loop, p1.match - p0.match, p1.solve - p0.solve,
          p1.nearChains - p0.nearChains, p1.nearChainsMatched - p0.nearChainsMatched});
    }
  }
  std::size_t no_odom_{0};
  std::size_t paused_scans_{0};  // scans that arrived while /slam_toolbox/pause was set
};

}  // namespace slam_toolbox

int main(int argc, char ** argv)
{
  const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() < 3) {
    std::fprintf(stderr,
      "Usage: %s <bag> <out_prefix> [--save] [--surface <file>] [--loop-surface <file>] "
      "[--viz] [--rate <factor>] [--start <seconds>] --ros-args ...\n", argv[0]);
    return 1;
  }
  bool save = false;
  bool viz = false;
  double rate = 0.0;
  double start_offset = 0.0;
  std::string surface;
  std::string loop_surface;
  for (std::size_t i = 3; i < args.size(); i++) {
    if (args[i] == "--save") {
      save = true;
    } else if (args[i] == "--surface" && i + 1 < args.size()) {
      surface = args[++i];
    } else if (args[i] == "--loop-surface" && i + 1 < args.size()) {
      loop_surface = args[++i];
    } else if (args[i] == "--viz") {
      viz = true;
    } else if (args[i] == "--rate" && i + 1 < args.size()) {
      rate = std::stod(args[++i]);
    } else if (args[i] == "--start" && i + 1 < args.size()) {
      start_offset = std::stod(args[++i]);
    } else {
      std::fprintf(stderr, "offline_replay: unknown argument %s\n", args[i].c_str());
      return 1;
    }
  }

  // Deep recursion when serializing large graphs, as in the slam_toolbox nodes.
  struct rlimit stack_limit;
  getrlimit(RLIMIT_STACK, &stack_limit);
  const rlim_t stack_size = 40000000;
  if (stack_limit.rlim_cur < stack_size) {
    stack_limit.rlim_cur = stack_size;
    setrlimit(RLIMIT_STACK, &stack_limit);
  }

  auto node = std::make_shared<slam_toolbox::OfflineReplay>(rclcpp::NodeOptions());
  node->configure();
  if (!surface.empty()) {
    node->surface_ = std::make_unique<std::ofstream>(surface);
    *node->surface_ << std::setprecision(9);
  }
  if (!loop_surface.empty()) {
    node->loop_surface_ = std::make_unique<std::ofstream>(loop_surface);
    *node->loop_surface_ << std::setprecision(6);
  }
  const int result = node->run(args[1], args[2], save, rate, start_offset);
  // std::_Exit below skips the streams' destructors; flush them here.
  if (node->surface_) {
    node->surface_->flush();
  }
  if (node->loop_surface_) {
    node->loop_surface_->flush();
  }
  if (viz && rclcpp::ok()) {
    std::fflush(nullptr);
    node->holdVisualization();
  }
  rclcpp::shutdown();
  // Everything is written and closed. Tearing the node down can race the base class's worker
  // threads ("pure virtual method called" from a half-destroyed OfflineReplay), so skip it.
  std::fflush(nullptr);
  std::_Exit(result);
}
