/*
 * slam_toolbox
 * Tests for online joint lidar<->base_link extrinsic calibration in CeresSolver.
 * See PLAN.md for the full design; this file focuses on the single highest-risk detail
 * called out there: the rotation reference used by SensorExtrinsicPoseGraph2dErrorTerm must be
 * the full composed sensor heading (yaw_a + yaw_e), not the robot heading alone, or the
 * extrinsic yaw becomes permanently unobservable with zero gradient -- silently, with no error.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "solvers/ceres_solver.hpp"

using karto::Pose2;
using karto::Name;
using karto::Matrix3;
using karto::LocalizedRangeScan;
using karto::LaserRangeFinder;
using karto::LaserRangeFinder_Custom;
using karto::SensorManager;
using karto::Vertex;
using karto::Edge;
using karto::LinkInfo;
using karto::RangeReadingsVector;

namespace
{

// Composes robotPose and offset exactly as karto::LocalizedRangeScan::GetSensorAt does:
// sensorPose = robotPose (+) offset, i.e. sensorPose.pos = robotPose.pos +
// R(robotPose.heading) * offset.pos, sensorPose.heading = robotPose.heading + offset.heading.
// (Verified against Transform::TransformPose in Karto.h -- see PLAN.md section 2.3.)
Pose2 ComposeSensorPose(const Pose2 & robotPose, const Pose2 & offset)
{
  const double c = std::cos(robotPose.GetHeading());
  const double s = std::sin(robotPose.GetHeading());
  const double x = robotPose.GetX() + c * offset.GetX() - s * offset.GetY();
  const double y = robotPose.GetY() + s * offset.GetX() + c * offset.GetY();
  return Pose2(x, y, robotPose.GetHeading() + offset.GetHeading());
}

Matrix3 SmallCovariance()
{
  Matrix3 cov;
  cov(0, 0) = 0.01;
  cov(1, 1) = 0.01;
  cov(2, 2) = 0.01;
  return cov;
}

LaserRangeFinder * makeAndRegisterLaser(const std::string & name, const Pose2 & nominalOffset)
{
  LaserRangeFinder * laser = LaserRangeFinder::CreateLaserRangeFinder(
    LaserRangeFinder_Custom, Name(name));
  laser->SetOffsetPose(nominalOffset);
  SensorManager::GetInstance()->RegisterSensor(laser, true /* override if already present */);
  return laser;
}

LocalizedRangeScan * makeScan(const Name & sensorName, int id, const Pose2 & correctedPose)
{
  LocalizedRangeScan * scan = new LocalizedRangeScan(sensorName, RangeReadingsVector());
  scan->SetUniqueId(id);
  scan->SetCorrectedPose(correctedPose);
  return scan;
}

// The correction limits default far out of the way: only the bounds test is about them.
rclcpp::Node::SharedPtr makeTestNode(
  const std::string & node_name, bool optimize_extrinsics,
  double prior_stddev_xy, double prior_stddev_yaw,
  double max_correction_xy = 1.0, double max_correction_yaw = 1.0)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("optimize_sensor_extrinsics", optimize_extrinsics),
    rclcpp::Parameter("extrinsic_prior_stddev_xy", prior_stddev_xy),
    rclcpp::Parameter("extrinsic_prior_stddev_yaw", prior_stddev_yaw),
    rclcpp::Parameter("extrinsic_max_correction_xy", max_correction_xy),
    rclcpp::Parameter("extrinsic_max_correction_yaw", max_correction_yaw),
  });
  auto node = std::make_shared<rclcpp::Node>(node_name, options);
  node->declare_parameter("debug_logging", false);
  return node;
}

// Builds a small chain graph: node i -> node i+1, with real (noise-free) sensor-frame
// measurements derived from robotPoses[i]/[i+1] composed with the given ground-truth
// extrinsic, and (when withOdometry) an independent robot-frame odometry measurement taken
// directly from robotPoses (in these noiseless synthetic tests, "odometry" and "true robot
// pose" are the same ground truth -- realistic EKF noise isn't modeled here). Feeds it through
// a CeresSolver configured with optimize_sensor_extrinsics true, anchored to nominalOffset, and
// returns the solver (caller owns cleanup of the scans/graph).
std::unique_ptr<solver_plugins::CeresSolver> runExtrinsicScenario(
  const std::string & sensorNameStr,
  const std::vector<Pose2> & robotPoses,
  const Pose2 & groundTruthExtrinsic,
  const Pose2 & nominalOffset,
  double prior_stddev_xy,
  double prior_stddev_yaw,
  std::vector<LocalizedRangeScan *> & scans_out,
  std::vector<Vertex<LocalizedRangeScan> *> & vertices_out,
  std::vector<Edge<LocalizedRangeScan> *> & edges_out,
  bool closeLoop = false,
  bool withOdometry = true,
  double max_correction_xy = 1.0,
  double max_correction_yaw = 1.0,
  const Pose2 * seed = nullptr,
  bool estimate_xy = true,
  bool estimate_yaw = true)
{
  makeAndRegisterLaser(sensorNameStr, nominalOffset);
  const Name sensorName(sensorNameStr);

  auto node = makeTestNode(
    sensorNameStr + "_node", true, prior_stddev_xy, prior_stddev_yaw, max_correction_xy,
    max_correction_yaw);

  auto solver = std::make_unique<solver_plugins::CeresSolver>();
  solver->Configure(node);
  if (seed) {
    // before the sensor's offset exists: kept for when the first edge creates it
    EXPECT_FALSE(solver->SetSensorOffsetCorrection(*seed));
  }
  EXPECT_TRUE(solver->SetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetXY, estimate_xy));
  EXPECT_TRUE(
    solver->SetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetYaw, estimate_yaw));

  for (size_t i = 0; i < robotPoses.size(); ++i) {
    LocalizedRangeScan * scan = makeScan(sensorName, static_cast<int>(i), robotPoses[i]);
    scans_out.push_back(scan);
    Vertex<LocalizedRangeScan> * vertex = new Vertex<LocalizedRangeScan>(scan);
    vertices_out.push_back(vertex);
    solver->AddNode(vertex);
  }

  auto addEdge = [&](size_t i, size_t j) {
    const Pose2 sensorPoseA = ComposeSensorPose(robotPoses[i], groundTruthExtrinsic);
    const Pose2 sensorPoseB = ComposeSensorPose(robotPoses[j], groundTruthExtrinsic);

    Edge<LocalizedRangeScan> * edge =
      new Edge<LocalizedRangeScan>(vertices_out[i], vertices_out[j]);
    edges_out.push_back(edge);

    LinkInfo * linkInfo = new LinkInfo(
      scans_out[i]->GetCorrectedPose(), scans_out[j]->GetCorrectedPose(), SmallCovariance());
    linkInfo->UpdateSensorFrame(sensorPoseA, sensorPoseB, SmallCovariance());
    if (withOdometry) {
      linkInfo->UpdateOdometryFrame(robotPoses[i], robotPoses[j]);
    }
    edge->SetLabel(linkInfo);

    solver->AddConstraint(edge);
  };

  for (size_t i = 0; i + 1 < robotPoses.size(); ++i) {
    addEdge(i, i + 1);
  }
  if (closeLoop && robotPoses.size() > 2) {
    // Without a loop closure, an open chain of sensor-frame-only edges has a genuine gauge
    // ambiguity: for *any* extrinsic value, there is a compensating set of intermediate node
    // poses that zeroes every edge residual exactly (conjugating each edge's robot-frame
    // relative motion by the extrinsic). Closing the loop back to the first (pinned) node
    // breaks that ambiguity and is what makes the extrinsic actually observable -- consistent
    // with PLAN.md's own integration-test guidance that calibration needs loop closure.
    addEdge(robotPoses.size() - 1, 0);
  }

  solver->Compute();
  return solver;
}

void cleanup(
  std::vector<LocalizedRangeScan *> & scans,
  std::vector<Vertex<LocalizedRangeScan> *> & vertices,
  std::vector<Edge<LocalizedRangeScan> *> & edges)
{
  for (auto * e : edges) {delete e;}
  for (auto * v : vertices) {delete v;}
  for (auto * s : scans) {delete s;}
}

}  // namespace

// Regression guard for the rotation-reference pitfall (PLAN.md section 2.3) AND for the
// extrinsic-observability fix (PLAN.md addendum): with genuine heading diversity (a small
// square path -- translation at four distinct headings) plus an independent odometry
// measurement per edge, the extrinsic estimate must converge close to the injected ground
// truth in x, y, AND yaw. No loop closure is needed here -- the odometry residual is what
// actually makes `ext` identifiable (see the addendum's AX=XB argument), not loop closure.
// If the sensor residual ever rotates by yaw_a alone instead of yaw_a + yaw_e, yaw_e gets zero
// gradient and this test's yaw assertion fails (it would stay pinned at the nominal/prior value
// instead).
TEST(CeresExtrinsicTest, DiverseMotionRecoversExtrinsic)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.10, -0.05, 0.15);
  const Pose2 nominalOffset(0.0, 0.0, 0.0);  // deliberately wrong, like an unmodeled mount error

  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "diverse_motion_lidar", robotPoses, groundTruthExtrinsic, nominalOffset,
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true);

  const auto & corrections = solver->GetSensorOffsetCorrections();
  ASSERT_EQ(corrections.count("diverse_motion_lidar"), 1u);
  const Pose2 & estimate = corrections.at("diverse_motion_lidar");

  EXPECT_NEAR(estimate.GetX(), groundTruthExtrinsic.GetX(), 1e-3);
  EXPECT_NEAR(estimate.GetY(), groundTruthExtrinsic.GetY(), 1e-3);
  EXPECT_NEAR(estimate.GetHeading(), groundTruthExtrinsic.GetHeading(), 1e-3);

  cleanup(scans, vertices, edges);
}

// Regression guard for the observability gap itself (PLAN.md addendum): the exact same diverse
// trajectory as above, but with no odometry edges at all. Without an independent robot-frame
// measurement, the sensor-frame-only residual has an exact SE(2) conjugation symmetry in `ext`
// (see the addendum's proof) and the estimate must NOT move off the nominal/prior -- if this
// assertion ever starts failing (estimate moving toward ground truth), something has silently
// introduced information that makes this test's premise wrong and needs investigating, not
// loosening.
TEST(CeresExtrinsicTest, WithoutOdometryExtrinsicStaysAtPrior)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.10, -0.05, 0.15);
  const Pose2 nominalOffset(0.0, 0.0, 0.0);

  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "no_odom_lidar", robotPoses, groundTruthExtrinsic, nominalOffset,
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/true, /*withOdometry=*/false);

  const auto & corrections = solver->GetSensorOffsetCorrections();
  ASSERT_EQ(corrections.count("no_odom_lidar"), 1u);
  const Pose2 & estimate = corrections.at("no_odom_lidar");

  // stayed close to the (wrong) nominal, nowhere near ground truth
  EXPECT_NEAR(estimate.GetX(), nominalOffset.GetX(), 1e-2);
  EXPECT_NEAR(estimate.GetY(), nominalOffset.GetY(), 1e-2);
  EXPECT_NEAR(estimate.GetHeading(), nominalOffset.GetHeading(), 1e-2);

  cleanup(scans, vertices, edges);
}

// Under pure in-place rotation (no translation at all between nodes), the extrinsic yaw is at
// best very weakly observable. With a tight prior, the estimate should stay much closer to the
// (wrong) nominal/prior yaw than to the ground truth -- i.e. it should not "solve" yaw from
// rotation-only motion the way it can from the diverse trajectory above.
TEST(CeresExtrinsicTest, PureRotationLeavesYawNearPrior)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(0.0, 0.0, M_PI_2),
    Pose2(0.0, 0.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.10, -0.05, 0.15);
  const Pose2 nominalOffset(0.0, 0.0, 0.0);

  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "pure_rotation_lidar", robotPoses, groundTruthExtrinsic, nominalOffset,
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/0.05,  // tight yaw prior
    scans, vertices, edges);

  const auto & corrections = solver->GetSensorOffsetCorrections();
  ASSERT_EQ(corrections.count("pure_rotation_lidar"), 1u);
  const Pose2 & estimate = corrections.at("pure_rotation_lidar");

  const double yaw_gt_distance = std::fabs(groundTruthExtrinsic.GetHeading() -
    nominalOffset.GetHeading());
  const double yaw_estimate_distance = std::fabs(estimate.GetHeading() -
    nominalOffset.GetHeading());
  // moved less than a third of the way from the (tight) prior toward ground truth
  EXPECT_LT(yaw_estimate_distance, 0.3 * yaw_gt_distance);

  cleanup(scans, vertices, edges);
}

// When extrinsic calibration is disabled, AddConstraint must take the legacy,
// extrinsic-independent path: no sensor offset parameter block is created, and
// GetSensorOffsetCorrections() stays empty.
TEST(CeresExtrinsicTest, DisabledFlagUsesLegacyPathAndReportsNoCorrections)
{
  const Name sensorName("disabled_lidar");
  makeAndRegisterLaser("disabled_lidar", Pose2(0.05, 0.0, 0.0));

  auto node = makeTestNode("disabled_node", /*optimize_extrinsics=*/false, 0.02, 0.02);
  solver_plugins::CeresSolver solver;
  solver.Configure(node);

  LocalizedRangeScan * scanA = makeScan(sensorName, 0, Pose2(0.0, 0.0, 0.0));
  LocalizedRangeScan * scanB = makeScan(sensorName, 1, Pose2(1.0, 0.0, 0.0));
  Vertex<LocalizedRangeScan> * vA = new Vertex<LocalizedRangeScan>(scanA);
  Vertex<LocalizedRangeScan> * vB = new Vertex<LocalizedRangeScan>(scanB);
  solver.AddNode(vA);
  solver.AddNode(vB);

  Edge<LocalizedRangeScan> * edge = new Edge<LocalizedRangeScan>(vA, vB);
  LinkInfo * linkInfo = new LinkInfo(
    scanA->GetCorrectedPose(), scanB->GetCorrectedPose(), SmallCovariance());
  linkInfo->UpdateSensorFrame(
    Pose2(0.0, 0.0, 0.0), Pose2(1.0, 0.0, 0.0), SmallCovariance());
  edge->SetLabel(linkInfo);
  solver.AddConstraint(edge);

  solver.Compute();

  EXPECT_TRUE(solver.GetSensorOffsetCorrections().empty());
  ASSERT_FALSE(solver.GetCorrections().empty());

  delete edge;
  delete vA;
  delete vB;
  delete scanA;
  delete scanB;
}

// Backward compatibility: an edge whose LinkInfo was built the old way (no UpdateSensorFrame
// call -- as any edge loaded from a pose graph serialized before this feature existed will be,
// per the BOOST_CLASS_VERSION gate in Mapper.h) must still be usable even with extrinsic
// calibration enabled: AddConstraint should fall back to the legacy residual instead of
// dereferencing missing sensor-frame data.
TEST(CeresExtrinsicTest, MissingSensorFrameDataFallsBackToLegacyPath)
{
  const Name sensorName("legacy_edge_lidar");
  makeAndRegisterLaser("legacy_edge_lidar", Pose2(0.05, 0.0, 0.0));

  auto node = makeTestNode("legacy_edge_node", /*optimize_extrinsics=*/true, 0.02, 0.02);
  solver_plugins::CeresSolver solver;
  solver.Configure(node);

  LocalizedRangeScan * scanA = makeScan(sensorName, 0, Pose2(0.0, 0.0, 0.0));
  LocalizedRangeScan * scanB = makeScan(sensorName, 1, Pose2(1.0, 0.0, 0.0));
  Vertex<LocalizedRangeScan> * vA = new Vertex<LocalizedRangeScan>(scanA);
  Vertex<LocalizedRangeScan> * vB = new Vertex<LocalizedRangeScan>(scanB);
  solver.AddNode(vA);
  solver.AddNode(vB);

  Edge<LocalizedRangeScan> * edge = new Edge<LocalizedRangeScan>(vA, vB);
  // old-style construction: UpdateSensorFrame is never called
  LinkInfo * linkInfo = new LinkInfo(
    scanA->GetCorrectedPose(), scanB->GetCorrectedPose(), SmallCovariance());
  ASSERT_FALSE(linkInfo->HasSensorFrameData());
  edge->SetLabel(linkInfo);

  solver.AddConstraint(edge);
  solver.Compute();

  EXPECT_TRUE(solver.GetSensorOffsetCorrections().empty());
  ASSERT_FALSE(solver.GetCorrections().empty());

  delete edge;
  delete vA;
  delete vB;
  delete scanA;
  delete scanB;
}

// extrinsic_max_correction_*: Ceres bounds hold the estimate within the limits of the nominal
// even where the data pulls it further (ground truth 0.10, -0.05, 0.15 from a nominal at 0).
TEST(CeresExtrinsicTest, BoundsHoldTheEstimate)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "bounded_lidar", robotPoses, Pose2(0.10, -0.05, 0.15), Pose2(0.0, 0.0, 0.0),
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true,
    /*max_correction_xy=*/0.02, /*max_correction_yaw=*/0.05);

  const Pose2 & estimate = solver->GetSensorOffsetCorrections().at("bounded_lidar");
  EXPECT_NEAR(estimate.GetX(), 0.02, 1e-6);
  EXPECT_NEAR(estimate.GetY(), -0.02, 1e-6);
  EXPECT_NEAR(estimate.GetHeading(), 0.05, 1e-6);

  cleanup(scans, vertices, edges);
}

// A reset (identity correction) after a solve puts the estimate back on the nominal and drops
// the covariance computed for the old estimate.
TEST(CeresExtrinsicTest, ResetRestoresTheNominal)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 nominalOffset(0.5, 0.0, 0.0);
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "reset_lidar", robotPoses, Pose2(0.55, 0.02, 0.05), nominalOffset,
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0, scans, vertices, edges);
  ASSERT_GT(std::abs(solver->GetSensorOffsetCorrections().at("reset_lidar").GetHeading()), 0.01);
  ASSERT_EQ(solver->GetSensorOffsetCovariances().count("reset_lidar"), 1u);

  EXPECT_TRUE(solver->SetSensorOffsetCorrection(Pose2()));
  const Pose2 & estimate = solver->GetSensorOffsetCorrections().at("reset_lidar");
  EXPECT_DOUBLE_EQ(estimate.GetX(), nominalOffset.GetX());
  EXPECT_DOUBLE_EQ(estimate.GetY(), nominalOffset.GetY());
  EXPECT_DOUBLE_EQ(estimate.GetHeading(), nominalOffset.GetHeading());
  EXPECT_EQ(solver->GetSensorOffsetCovariances().count("reset_lidar"), 0u);

  cleanup(scans, vertices, edges);
}

// A seed (a persisted correction) given before the sensor is known starts its estimate at
// nominal * seed, clamped to the limits, from the moment the sensor is initialized.
TEST(CeresExtrinsicTest, SeedStartsTheEstimateClamped)
{
  auto node = makeTestNode("seed_node", true, 0.08, 0.06, 0.05, 0.035);
  solver_plugins::CeresSolver solver;
  solver.Configure(node);

  EXPECT_FALSE(solver.SetSensorOffsetCorrection(Pose2(0.03, 0.2, -0.1)));
  const Pose2 nominal(0.51, 0.0, M_PI_2);
  Pose2 estimate;
  ASSERT_TRUE(solver.InitializeSensorOffset("seed_lidar", nominal, estimate));
  // the seed is in the sensor frame: (0.03, 0.2) turned by the nominal's 90 degrees is
  // (-0.2, 0.03) in the robot frame, x clamped to 0.05; yaw clamped to 0.035
  EXPECT_NEAR(estimate.GetX(), 0.51 - 0.05, 1e-9);
  EXPECT_NEAR(estimate.GetY(), 0.03, 1e-9);
  EXPECT_NEAR(estimate.GetHeading(), M_PI_2 - 0.035, 1e-9);
  EXPECT_EQ(solver.GetSensorNominalOffsets().at("seed_lidar").GetX(), nominal.GetX());

  // without calibration there is nothing to seed
  auto off_node = makeTestNode("seed_off_node", false, 0.08, 0.06);
  solver_plugins::CeresSolver off_solver;
  off_solver.Configure(off_node);
  EXPECT_FALSE(off_solver.InitializeSensorOffset("seed_lidar", nominal, estimate));
}

// Seeded close to the truth, the solve still converges to it (the prior stays on the nominal,
// so the seed is only the start).
TEST(CeresExtrinsicTest, SolveConvergesFromASeed)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.03, 0.01, 0.015);
  const Pose2 seed(0.02, 0.0, 0.01);
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "seeded_lidar", robotPoses, groundTruthExtrinsic, Pose2(0.0, 0.0, 0.0),
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true,
    /*max_correction_xy=*/0.05, /*max_correction_yaw=*/0.035, &seed);

  const Pose2 & estimate = solver->GetSensorOffsetCorrections().at("seeded_lidar");
  EXPECT_NEAR(estimate.GetX(), groundTruthExtrinsic.GetX(), 1e-3);
  EXPECT_NEAR(estimate.GetY(), groundTruthExtrinsic.GetY(), 1e-3);
  EXPECT_NEAR(estimate.GetHeading(), groundTruthExtrinsic.GetHeading(), 1e-3);

  cleanup(scans, vertices, edges);
}

// With estimation off the seeded extrinsic is used but never moved by a solve; switched on, the
// same graph then learns it.
TEST(CeresExtrinsicTest, FrozenEstimateStaysAtTheSeed)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.03, 0.01, 0.015);
  const Pose2 seed(0.02, 0.0, 0.01);
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "frozen_lidar", robotPoses, groundTruthExtrinsic, Pose2(0.0, 0.0, 0.0),
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true,
    /*max_correction_xy=*/0.05, /*max_correction_yaw=*/0.035, &seed, /*estimate_xy=*/false,
    /*estimate_yaw=*/false);
  EXPECT_FALSE(solver->GetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetXY));
  EXPECT_FALSE(solver->GetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetYaw));
  const Pose2 frozen = solver->GetSensorOffsetCorrections().at("frozen_lidar");
  EXPECT_DOUBLE_EQ(frozen.GetX(), seed.GetX());
  EXPECT_DOUBLE_EQ(frozen.GetY(), seed.GetY());
  EXPECT_DOUBLE_EQ(frozen.GetHeading(), seed.GetHeading());
  // the poses still solve
  EXPECT_EQ(solver->GetCorrections().size(), robotPoses.size());

  EXPECT_TRUE(solver->SetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetXY, true));
  EXPECT_TRUE(solver->SetSensorOffsetEstimation(karto::ScanSolver::SensorOffsetYaw, true));
  solver->Compute();
  const Pose2 & learned = solver->GetSensorOffsetCorrections().at("frozen_lidar");
  EXPECT_NEAR(learned.GetX(), groundTruthExtrinsic.GetX(), 1e-3);
  EXPECT_NEAR(learned.GetY(), groundTruthExtrinsic.GetY(), 1e-3);
  EXPECT_NEAR(learned.GetHeading(), groundTruthExtrinsic.GetHeading(), 1e-3);

  cleanup(scans, vertices, edges);
}

// estimate_sensor_extrinsic_xy off: x/y stay exactly on the nominal (no seed here) while the
// yaw is learned, and only the yaw has a variance.
TEST(CeresExtrinsicTest, XyOffLearnsTheYawOnly)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 nominal(0.5, 0.0, 0.0);
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "yaw_only_lidar", robotPoses, Pose2(0.5, 0.0, 0.015), nominal,
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true,
    /*max_correction_xy=*/0.05, /*max_correction_yaw=*/0.035, nullptr, /*estimate_xy=*/false,
    /*estimate_yaw=*/true);

  const Pose2 & estimate = solver->GetSensorOffsetCorrections().at("yaw_only_lidar");
  EXPECT_DOUBLE_EQ(estimate.GetX(), nominal.GetX());
  EXPECT_DOUBLE_EQ(estimate.GetY(), nominal.GetY());
  EXPECT_NEAR(estimate.GetHeading(), 0.015, 1e-3);
  const Eigen::Matrix3d & cov = solver->GetSensorOffsetCovariances().at("yaw_only_lidar");
  EXPECT_EQ(cov(0, 0), 0.0);
  EXPECT_EQ(cov(1, 1), 0.0);
  EXPECT_GT(cov(2, 2), 0.0);

  cleanup(scans, vertices, edges);
}

// estimate_sensor_extrinsic_yaw off: the yaw stays exactly at the seed while x/y are learned.
TEST(CeresExtrinsicTest, YawOffLearnsTheXyOnly)
{
  const std::vector<Pose2> robotPoses = {
    Pose2(0.0, 0.0, 0.0),
    Pose2(2.0, 0.0, 0.0),
    Pose2(2.0, 2.0, M_PI_2),
    Pose2(0.0, 2.0, M_PI),
    Pose2(0.0, 0.0, -M_PI_2),
  };
  const Pose2 groundTruthExtrinsic(0.03, 0.01, 0.015);
  const Pose2 seed(0.0, 0.0, 0.015);  // the true yaw, known
  std::vector<LocalizedRangeScan *> scans;
  std::vector<Vertex<LocalizedRangeScan> *> vertices;
  std::vector<Edge<LocalizedRangeScan> *> edges;

  auto solver = runExtrinsicScenario(
    "xy_only_lidar", robotPoses, groundTruthExtrinsic, Pose2(0.0, 0.0, 0.0),
    /*prior_stddev_xy=*/1.0, /*prior_stddev_yaw=*/1.0,
    scans, vertices, edges, /*closeLoop=*/false, /*withOdometry=*/true,
    /*max_correction_xy=*/0.05, /*max_correction_yaw=*/0.035, &seed, /*estimate_xy=*/true,
    /*estimate_yaw=*/false);

  const Pose2 & estimate = solver->GetSensorOffsetCorrections().at("xy_only_lidar");
  EXPECT_DOUBLE_EQ(estimate.GetHeading(), seed.GetHeading());
  EXPECT_NEAR(estimate.GetX(), groundTruthExtrinsic.GetX(), 1e-3);
  EXPECT_NEAR(estimate.GetY(), groundTruthExtrinsic.GetY(), 1e-3);
  EXPECT_EQ(solver->GetSensorOffsetCovariances().at("xy_only_lidar")(2, 2), 0.0);

  cleanup(scans, vertices, edges);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
