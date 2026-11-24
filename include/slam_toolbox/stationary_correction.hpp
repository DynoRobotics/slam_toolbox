// Copyright (C) Husqvarna Construction - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#ifndef SLAM_TOOLBOX__STATIONARY_CORRECTION_HPP_
#define SLAM_TOOLBOX__STATIONARY_CORRECTION_HPP_

// Stationary correction: while the robot stands still, no scan is added to the graph (not even
// by the minimum_time_interval timer); instead one every `interval` is only matched
// (karto::Mapper::MatchOnly), to keep map->odom following the EKF's heading drift (gyro bias at
// rest). A scan of people at the machine or a covered lidar added while standing still stays in
// the map for good, and hours of them crowd out the running scans and grow the graph at one spot.
// Often, so each correction stays small: the matcher estimates the lidar's pose, and a large
// heading correction about the robot's centre leaks into the position through the lidar's offset;
// and drift left uncorrected for long runs out of the matcher's angle window, past recovery until
// the robot moves.
//
// Stationary: over the last 5-10 s the odom base moves no faster than max_speed and turns no
// faster than max_drift_rate. Rates, not totals: drift, which only turns the heading and never
// faster than the gyro bias, never makes the robot count as moving however long it lasts, and a
// stop anywhere is stationary at once, with no scan added on arriving. Measured on the base, not
// the sensor: a heading change swings the lidar's offset sideways.
//
// Only the heading is corrected: drift at rest only turns the heading, while the match's position
// can be ambiguous (along a corridor) and the robot has not moved; the position error from the
// drive there is corrected by the next scan added. A match is accepted when it is plausible as
// drift, from the last accepted pose:
//   - response at least min_response,
//   - heading change at most heading_floor + max_drift_rate * (time since the last accepted
//     correction or graph scan), capped at heading_cap,
//   - position change at most max_translation plus translation_per_meter of the odom distance
//     travelled since the last accepted correction or graph scan (a stop away from the last graph
//     scan carries the odometry error of the way there, and the matcher a small bias of its own),
//     (a position far off means the scene is not what the map holds: people, a covered lidar),
// and its heading agrees with the previous attempt's (within agree_heading): a static scene
// matches the same twice, people moving about do not. Otherwise the last accepted pose stays,
// until the robot moves and a scan is added as usual.

#include <optional>

#include "karto_sdk/Karto.h"

namespace slam_toolbox
{

struct StationaryCorrectionParams
{
  bool enabled{false};
  double interval{0.0};             // s
  double max_speed{0.0};            // m/s
  double max_drift_rate{0.0};       // rad/s
  double heading_floor{0.0};        // rad
  double heading_cap{0.0};          // rad
  double max_translation{0.0};      // m
  double translation_per_meter{0.0};
  double agree_heading{0.0};        // rad
  double min_response{0.0};
};

class StationaryCorrection
{
public:
  enum class Reason
  {
    Accepted,
    NoMatch,       // nothing to match against yet
    LowResponse,
    Heading,       // heading change beyond the drift limit
    Translation,
    FirstMatch,    // plausible, waiting for a second one to agree
    Disagree,      // plausible, but not where the previous one was
  };

  struct Verdict
  {
    bool accepted{false};
    Reason reason{Reason::NoMatch};
    double heading_limit{0.0};      // rad
    double translation_limit{0.0};  // m
  };

  explicit StationaryCorrection(const StationaryCorrectionParams & params)
  : params_(params) {}

  const StationaryCorrectionParams & params() const {return params_;}

  // A scan was added to the graph at this odom base pose and time: the reference for the drift
  // limit and the correction interval.
  void onGraphScan(const karto::Pose2 & odom_base, double t);

  // Whether the robot stands still at this odom base pose. False before the first graph scan.
  bool isStationary(const karto::Pose2 & odom_base, double t);

  // Whether a correction is due: `interval` since the last graph scan or attempt.
  bool attemptDue(double t) const;

  // Judges one MatchOnly result (robot poses in the map frame; response < 0: no match) for the
  // scan at this odom base pose, and records the attempt.
  Verdict evaluate(
    const karto::Pose2 & predicted, const karto::Pose2 & matched, double response,
    const karto::Pose2 & odom_base, double t);

  // Slam was paused (the lidar may have been moved or covered): the next correction needs two
  // fresh matches to agree. Being stationary is still judged against the last graph scan, so a
  // robot that stood still through the pause gets no graph scan when it ends.
  void interrupt() {candidate_.reset();}

  // Forgets everything: the graph was replaced, or time went back.
  void reset();

  // The pose an accepted match is applied as: its heading at the predicted position.
  static karto::Pose2 applied(const karto::Pose2 & predicted, const karto::Pose2 & matched)
  {
    return karto::Pose2(predicted.GetX(), predicted.GetY(), matched.GetHeading());
  }

  static const char * toString(Reason reason);

private:
  StationaryCorrectionParams params_;
  bool have_graph_scan_{false};
  // Speed and turn-rate references: odom base pose and time, renewed every few seconds; rates are
  // measured from the older one, over 5-10 s.
  karto::Pose2 rate_pose_, rate_prev_pose_;
  double rate_t_{0.0};
  std::optional<double> rate_prev_t_;
  double last_attempt_t_{0.0};   // last graph scan or attempt
  double last_accepted_t_{0.0};  // last accepted correction or graph scan
  karto::Pose2 last_accepted_odom_;  // odom base pose then
  std::optional<karto::Pose2> candidate_;  // the previous attempt's match, if plausible
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__STATIONARY_CORRECTION_HPP_
