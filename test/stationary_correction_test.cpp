// Copyright (C) Husqvarna Construction - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include "slam_toolbox/stationary_correction.hpp"

#include <gtest/gtest.h>

#include <cmath>

using karto::Pose2;
using slam_toolbox::StationaryCorrection;
using slam_toolbox::StationaryCorrectionParams;
using Reason = StationaryCorrection::Reason;

namespace
{
constexpr double kDeg = M_PI / 180.0;

StationaryCorrectionParams params()
{
  StationaryCorrectionParams p;
  p.enabled = true;
  p.interval = 10.0;
  p.max_speed = 0.002;
  p.max_drift_rate = 0.01;
  p.heading_floor = 0.5 * kDeg;
  p.heading_cap = 10.0 * kDeg;
  p.max_translation = 0.08;
  p.translation_per_meter = 0.1;
  p.agree_heading = 0.3 * kDeg;
  p.min_response = 0.1;
  return p;
}

Pose2 at(double heading_deg, double x = 0.0) {return Pose2(x, 0.0, heading_deg * kDeg);}
}  // namespace

TEST(StationaryCorrection, NotStationaryBeforeAGraphScan)
{
  StationaryCorrection c(params());
  EXPECT_FALSE(c.isStationary(at(0), 1.0));
  EXPECT_FALSE(c.attemptDue(100.0));
}

TEST(StationaryCorrection, HoursOfDriftStayStationary)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  // 0.11 deg/s, uncompensated, for two hours: over 790 deg in total.
  for (double t = 0.1; t < 7200.0; t += 0.1) {
    ASSERT_TRUE(c.isStationary(at(0.11 * t), t)) << t;
  }
}

TEST(StationaryCorrection, RealTurnOrMoveIsNotStationary)
{
  StationaryCorrection turn(params());
  turn.onGraphScan(at(0), 0.0);
  for (double t = 0.1; t < 3600.0; t += 0.1) {
    turn.isStationary(at(0), t);
  }
  // A turn at 10 deg/s after an hour shows within a second.
  EXPECT_FALSE(turn.isStationary(at(10), 3601.0));

  StationaryCorrection move(params());
  move.onGraphScan(at(0), 0.0);
  EXPECT_TRUE(move.isStationary(at(0, 0.001), 1.0));
  EXPECT_FALSE(move.isStationary(at(0, 0.03), 1.2));  // 3 cm: a start shows at once
  // The slowest grinding speed, 5 mm/s, is moving.
  StationaryCorrection creep(params());
  creep.onGraphScan(at(0), 0.0);
  bool moving = false;
  for (double t = 0.1; t < 20.0; t += 0.1) {
    moving |= !creep.isStationary(at(0, 0.005 * t), t);
  }
  EXPECT_TRUE(moving);
}

TEST(StationaryCorrection, AttemptsEveryInterval)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  EXPECT_FALSE(c.attemptDue(9.9));
  EXPECT_TRUE(c.attemptDue(10.0));
  c.evaluate(at(0), at(0), 0.5, at(0), 10.0);
  EXPECT_FALSE(c.attemptDue(19.9));
  EXPECT_TRUE(c.attemptDue(20.0));
}

TEST(StationaryCorrection, AcceptsOnceTwoMatchesAgree)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  EXPECT_EQ(c.evaluate(at(0), at(1), 0.5, at(0), 10.0).reason, Reason::FirstMatch);
  const auto v = c.evaluate(at(0), at(1.1), 0.5, at(0), 20.0);
  EXPECT_TRUE(v.accepted);
  EXPECT_EQ(v.reason, Reason::Accepted);
}

TEST(StationaryCorrection, DisagreeingMatchesWait)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  c.evaluate(at(0), at(1), 0.5, at(0), 10.0);
  EXPECT_EQ(c.evaluate(at(0), at(2), 0.5, at(0), 20.0).reason, Reason::Disagree);
  EXPECT_TRUE(c.evaluate(at(0), at(2.1), 0.5, at(0), 30.0).accepted);  // agrees with the last one
}

TEST(StationaryCorrection, RejectsImplausibleMatches)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  EXPECT_EQ(c.evaluate(at(0), at(0), -1.0, at(0), 10.0).reason, Reason::NoMatch);
  EXPECT_EQ(c.evaluate(at(0), at(0), 0.05, at(0), 20.0).reason, Reason::LowResponse);
  EXPECT_EQ(c.evaluate(at(0), at(15), 0.5, at(0), 30.0).reason, Reason::Heading);
  EXPECT_EQ(c.evaluate(at(0), at(0, 0.1), 0.5, at(0), 40.0).reason, Reason::Translation);
  // A rejection clears the candidate: the next plausible match is a first one again.
  EXPECT_EQ(c.evaluate(at(0), at(0), 0.5, at(0), 50.0).reason, Reason::FirstMatch);
}

TEST(StationaryCorrection, TranslationLimitAllowsForTheDriveSinceTheLastAcceptance)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  // Stopped 0.3 m on: 0.08 + 0.1 * 0.3 = 0.11 m allowed.
  EXPECT_EQ(c.evaluate(at(0), at(0, 0.1), 0.5, at(0, 0.3), 10.0).reason, Reason::FirstMatch);
  EXPECT_EQ(c.evaluate(at(0), at(0, 0.12), 0.5, at(0, 0.3), 20.0).reason, Reason::Translation);
}

TEST(StationaryCorrection, HeadingLimitGrowsSinceLastAcceptanceUpToTheCap)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  const auto early = c.evaluate(at(0), at(0), 0.5, at(0), 1.0);
  EXPECT_NEAR(early.heading_limit, 0.5 * kDeg + 0.01, 1e-9);
  const auto late = c.evaluate(at(0), at(0), 0.5, at(0), 1000.0);
  EXPECT_NEAR(late.heading_limit, 10.0 * kDeg, 1e-9);
  ASSERT_TRUE(late.accepted);
  // Measured from the acceptance at 1000 s again.
  EXPECT_NEAR(c.evaluate(at(0), at(0), 0.5, at(0), 1001.0).heading_limit, 0.5 * kDeg + 0.01, 1e-9);
}

TEST(StationaryCorrection, InterruptNeedsTwoFreshMatchesButStaysStationary)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  c.evaluate(at(0), at(1), 0.5, at(0), 10.0);
  c.interrupt();
  EXPECT_TRUE(c.isStationary(at(0), 15.0));
  EXPECT_EQ(c.evaluate(at(0), at(1), 0.5, at(0), 20.0).reason, Reason::FirstMatch);
  EXPECT_TRUE(c.evaluate(at(0), at(1), 0.5, at(0), 30.0).accepted);
}

TEST(StationaryCorrection, StopAwayFromTheLastGraphScanIsStationary)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  for (double t = 0.1; t < 3.0; t += 0.1) {
    c.isStationary(at(0, 0.1 * t), t);  // drives on 0.3 m at 0.1 m/s
  }
  bool stationary = false;
  for (double t = 3.0; t < 12.0; t += 0.1) {
    stationary = c.isStationary(at(0, 0.3), t);  // and stops there
  }
  EXPECT_TRUE(stationary);
}

TEST(StationaryCorrection, MillimetresOfEkfJitterStayStationary)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  for (double t = 0.1; t < 600.0; t += 0.1) {
    const double jitter = 0.003 * std::sin(7.0 * t);  // +-3 mm, fast
    ASSERT_TRUE(c.isStationary(at(0, jitter), t)) << t;
  }
}

TEST(StationaryCorrection, AgreementIsOnHeadingOnly)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 0.0);
  c.evaluate(at(0), at(1, 0.0), 0.5, at(0), 10.0);
  // Along a corridor the position may jump between cells; the heading still agrees.
  EXPECT_TRUE(c.evaluate(at(0), at(1, 0.05), 0.5, at(0), 20.0).accepted);
  const Pose2 applied = StationaryCorrection::applied(at(0, 0.01), at(1, 0.05));
  EXPECT_DOUBLE_EQ(applied.GetX(), 0.01);
  EXPECT_DOUBLE_EQ(applied.GetHeading(), 1 * kDeg);
}

TEST(StationaryCorrection, TimeGoingBackResets)
{
  StationaryCorrection c(params());
  c.onGraphScan(at(0), 100.0);
  EXPECT_TRUE(c.isStationary(at(0), 101.0));
  EXPECT_FALSE(c.isStationary(at(0), 50.0));
  EXPECT_FALSE(c.attemptDue(1000.0));
}
