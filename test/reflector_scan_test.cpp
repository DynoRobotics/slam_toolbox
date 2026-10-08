// Reflector data on LocalizedRangeScan (REFLECTOR_SLAM.md stage 2): weights per reading,
// world-frame reflector points, and serialization.

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <vector>

#include "karto_sdk/Karto.h"

using karto::LaserRangeFinder;
using karto::LaserRangeFinder_Custom;
using karto::LocalizedRangeScan;
using karto::Name;
using karto::Pose2;
using karto::RangeReadingsVector;
using karto::SensorManager;

namespace
{

constexpr double kPi = 3.14159265358979323846;

// Five beams at -90, -45, 0, 45, 90 deg.
LaserRangeFinder * makeLaser(const std::string & name)
{
  LaserRangeFinder * laser = LaserRangeFinder::CreateLaserRangeFinder(
    LaserRangeFinder_Custom, Name(name));
  laser->SetOffsetPose(Pose2(0.0, 0.0, 0.0));
  laser->SetMinimumRange(0.1);
  laser->SetMaximumRange(20.0);
  laser->SetRangeThreshold(10.0);
  laser->SetMinimumAngle(-kPi / 2.0);
  laser->SetMaximumAngle(kPi / 2.0);
  laser->SetAngularResolution(kPi / 4.0);
  SensorManager::GetInstance()->RegisterSensor(laser, true);
  return laser;
}

}  // namespace

TEST(ReflectorScan, NoWeightsNoReflectorPoints)
{
  makeLaser("laser_none");
  LocalizedRangeScan scan(Name("laser_none"), RangeReadingsVector{2.0, 2.0, 2.0, 2.0, 2.0});
  scan.SetCorrectedPose(Pose2(0.0, 0.0, 0.0));
  EXPECT_EQ(scan.GetPointReadings(true).size(), 5u);
  EXPECT_TRUE(scan.GetReflectorPoints().empty());
  EXPECT_TRUE(scan.GetReflectorPointWeights().empty());
}

TEST(ReflectorScan, ReflectorPointsInWorldFrame)
{
  makeLaser("laser_world");
  // Beam 4 is beyond the range threshold: not a point reading, so not a reflector point.
  LocalizedRangeScan scan(Name("laser_world"), RangeReadingsVector{2.0, 3.0, 4.0, 2.0, 15.0});
  scan.SetReflectorWeights({0.0, 0.0, 1.0, 0.5, 1.0});
  scan.SetCorrectedPose(Pose2(1.0, 2.0, kPi / 2.0));

  const auto & points = scan.GetReflectorPoints();
  const auto & weights = scan.GetReflectorPointWeights();
  ASSERT_EQ(points.size(), 2u);
  ASSERT_EQ(weights.size(), 2u);
  // Beam 2 points straight ahead, which is +y in the world for a robot facing +y.
  EXPECT_NEAR(points[0].GetX(), 1.0, 1e-9);
  EXPECT_NEAR(points[0].GetY(), 6.0, 1e-9);
  EXPECT_DOUBLE_EQ(weights[0], 1.0);
  // Beam 3 at +45 deg from heading +90 deg: 135 deg in the world.
  EXPECT_NEAR(points[1].GetX(), 1.0 + 2.0 * std::cos(3.0 * kPi / 4.0), 1e-9);
  EXPECT_NEAR(points[1].GetY(), 2.0 + 2.0 * std::sin(3.0 * kPi / 4.0), 1e-9);
  EXPECT_DOUBLE_EQ(weights[1], 0.5);

  // Moving the scan moves its reflector points.
  scan.SetCorrectedPose(Pose2(0.0, 0.0, 0.0));
  EXPECT_NEAR(scan.GetReflectorPoints()[0].GetX(), 4.0, 1e-9);
  EXPECT_NEAR(scan.GetReflectorPoints()[0].GetY(), 0.0, 1e-9);
}

TEST(ReflectorScan, SerializationKeepsReflectorData)
{
  makeLaser("laser_serial");
  LocalizedRangeScan * scan =
    new LocalizedRangeScan(Name("laser_serial"), RangeReadingsVector{2.0, 3.0, 4.0, 2.0, 2.0});
  scan->SetReflectorWeights({0.0, 1.0, 0.0, 0.0, 0.25});
  scan->SetCorrectedPose(Pose2(0.5, -0.5, 0.3));
  const auto expected = scan->GetReflectorPoints();
  ASSERT_EQ(expected.size(), 2u);

  std::stringstream buffer;
  {
    boost::archive::binary_oarchive out(buffer);
    out << scan;
  }
  LocalizedRangeScan * loaded = nullptr;
  {
    boost::archive::binary_iarchive in(buffer);
    in >> loaded;
  }
  ASSERT_NE(loaded, nullptr);
  EXPECT_EQ(loaded->GetReflectorWeights(), scan->GetReflectorWeights());
  ASSERT_EQ(loaded->GetReflectorPoints().size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(loaded->GetReflectorPoints()[i].GetX(), expected[i].GetX(), 1e-12);
    EXPECT_NEAR(loaded->GetReflectorPoints()[i].GetY(), expected[i].GetY(), 1e-12);
  }
  EXPECT_EQ(loaded->GetReflectorPointWeights(), scan->GetReflectorPointWeights());
  delete loaded;
  delete scan;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
