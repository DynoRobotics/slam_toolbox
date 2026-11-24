// Copyright (C) Husqvarna Construction - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include "slam_toolbox/stationary_correction.hpp"

#include <algorithm>
#include <cmath>

namespace slam_toolbox
{

namespace
{
// Speed and turn rate are measured from a reference odom pose renewed every kRateWindow (s), the one
// before the current, so over kRateWindow to twice that: recent enough that a real move after hours
// at rest is not averaged away (or hours of drift wrapped), and long enough that the EKF's
// millimetres of jitter at rest do not read as moving. A real start still shows within a fraction
// of a second: the distance it covers is divided by the window, not by the time it took.
constexpr double kRateWindow = 5.0;

double headingDifference(const karto::Pose2 & a, const karto::Pose2 & b)
{
  return std::fabs(karto::math::NormalizeAngle(a.GetHeading() - b.GetHeading()));
}

double distance(const karto::Pose2 & a, const karto::Pose2 & b)
{
  return std::hypot(a.GetX() - b.GetX(), a.GetY() - b.GetY());
}
}  // namespace

void StationaryCorrection::onGraphScan(const karto::Pose2 & odom_base, double t)
{
  have_graph_scan_ = true;
  rate_pose_ = odom_base;
  rate_t_ = t;
  rate_prev_t_.reset();
  last_attempt_t_ = t;
  last_accepted_t_ = t;
  last_accepted_odom_ = odom_base;
  candidate_.reset();
}

bool StationaryCorrection::isStationary(const karto::Pose2 & odom_base, double t)
{
  if (!have_graph_scan_) {
    return false;
  }
  if (t < rate_t_) {  // time went back (bag restart): start over from the next graph scan
    reset();
    return false;
  }
  if (t - rate_t_ >= kRateWindow) {
    rate_prev_pose_ = rate_pose_;
    rate_prev_t_ = rate_t_;
    rate_pose_ = odom_base;
    rate_t_ = t;
  }
  const karto::Pose2 & from = rate_prev_t_ ? rate_prev_pose_ : rate_pose_;
  const double window = std::max(t - (rate_prev_t_ ? *rate_prev_t_ : rate_t_), kRateWindow);
  return distance(odom_base, from) / window <= params_.max_speed &&
         headingDifference(odom_base, from) / window <= params_.max_drift_rate;
}

bool StationaryCorrection::attemptDue(double t) const
{
  return have_graph_scan_ && t - last_attempt_t_ >= params_.interval;
}

StationaryCorrection::Verdict StationaryCorrection::evaluate(
  const karto::Pose2 & predicted, const karto::Pose2 & matched, double response,
  const karto::Pose2 & odom_base, double t)
{
  last_attempt_t_ = t;
  Verdict verdict;
  verdict.heading_limit = std::min(
    params_.heading_cap,
    params_.heading_floor + params_.max_drift_rate * std::max(t - last_accepted_t_, 0.0));
  verdict.translation_limit = params_.max_translation +
    params_.translation_per_meter * distance(odom_base, last_accepted_odom_);
  const auto reject = [&](Reason reason) {
      candidate_.reset();
      verdict.reason = reason;
      return verdict;
    };
  if (response < 0.0) {
    return reject(Reason::NoMatch);
  }
  if (response < params_.min_response) {
    return reject(Reason::LowResponse);
  }
  if (headingDifference(matched, predicted) > verdict.heading_limit) {
    return reject(Reason::Heading);
  }
  if (distance(matched, predicted) > verdict.translation_limit) {
    return reject(Reason::Translation);
  }
  const bool agrees = candidate_ &&
    headingDifference(matched, *candidate_) <= params_.agree_heading;
  const bool first = !candidate_;
  candidate_ = matched;
  if (!agrees) {
    verdict.reason = first ? Reason::FirstMatch : Reason::Disagree;
    return verdict;
  }
  verdict.accepted = true;
  verdict.reason = Reason::Accepted;
  last_accepted_t_ = t;
  last_accepted_odom_ = odom_base;
  return verdict;
}

void StationaryCorrection::reset()
{
  have_graph_scan_ = false;
  candidate_.reset();
  rate_pose_ = karto::Pose2();
  rate_prev_pose_ = karto::Pose2();
  rate_prev_t_.reset();
  rate_t_ = 0.0;
  last_attempt_t_ = 0.0;
  last_accepted_t_ = 0.0;
  last_accepted_odom_ = karto::Pose2();
}

const char * StationaryCorrection::toString(Reason reason)
{
  switch (reason) {
    case Reason::Accepted: return "accepted";
    case Reason::NoMatch: return "no_match";
    case Reason::LowResponse: return "low_response";
    case Reason::Heading: return "heading";
    case Reason::Translation: return "translation";
    case Reason::FirstMatch: return "first_match";
    case Reason::Disagree: return "disagree";
  }
  return "unknown";
}

}  // namespace slam_toolbox
