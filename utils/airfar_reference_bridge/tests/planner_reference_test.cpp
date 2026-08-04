#include <cmath>

#include <gtest/gtest.h>

#include <airfar_reference_bridge/arrival_contract.h>
#include <airfar_reference_bridge/planner_reference.h>

namespace {

using airfar_reference_bridge::PlannerReferenceBuilder;
using airfar_reference_bridge::PlannerReferenceInput;
using airfar_reference_bridge::ReferenceMode;
using airfar_reference_bridge::IsWithinGoalTolerance;

constexpr double kTolerance = 1e-9;

TEST(ArrivalContract, UsesThreeDimensionalDistance) {
  EXPECT_FALSE(IsWithinGoalTolerance(
      0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.4));
  EXPECT_TRUE(IsWithinGoalTolerance(
      0.1, 0.2, 0.1, 0.0, 0.0, 0.0, 0.4));
}

TEST(PlannerReferenceBuilder, TrackReferenceIsWorldFrameKinematics) {
  PlannerReferenceBuilder builder;
  PlannerReferenceInput input;
  input.mode = ReferenceMode::kTrackPath;
  input.track_x = 4.0;
  input.track_y = 5.0;
  input.track_z = 6.0;
  input.track_yaw = M_PI / 2.0;
  input.path_speed = 2.0;
  input.path_vertical_velocity = 0.3;
  input.path_curvature = 0.25;
  input.path_yaw_rate = 0.5;

  const auto output = builder.Build(input);

  EXPECT_NEAR(output.position.x, 4.0, kTolerance);
  EXPECT_NEAR(output.position.y, 5.0, kTolerance);
  EXPECT_NEAR(output.position.z, 6.0, kTolerance);
  EXPECT_NEAR(output.velocity.x, 0.0, kTolerance);
  EXPECT_NEAR(output.velocity.y, 2.0, kTolerance);
  EXPECT_NEAR(output.velocity.z, 0.3, kTolerance);
  EXPECT_NEAR(output.acceleration.x, -1.0, kTolerance);
  EXPECT_NEAR(output.acceleration.y, 0.0, kTolerance);
  EXPECT_NEAR(output.yaw, M_PI / 2.0, kTolerance);
  EXPECT_NEAR(output.yaw_rate, 0.5, kTolerance);
}

TEST(PlannerReferenceBuilder, RotateInPlaceUsesStablePositionAndAbsoluteYaw) {
  PlannerReferenceBuilder builder;
  PlannerReferenceInput input;
  input.mode = ReferenceMode::kRotateInPlace;
  input.stamp = 1.0;
  input.vehicle_x = 1.0;
  input.vehicle_y = 2.0;
  input.vehicle_z = 3.0;
  input.vehicle_yaw = -0.2;
  input.goal_x = 4.0;
  input.goal_y = 6.0;

  const auto first = builder.Build(input);
  input.stamp = 2.0;
  input.vehicle_x = -10.0;
  input.vehicle_y = -20.0;
  const auto second = builder.Build(input);

  EXPECT_NEAR(first.position.x, 1.0, kTolerance);
  EXPECT_NEAR(first.position.y, 2.0, kTolerance);
  EXPECT_NEAR(first.position.z, 3.0, kTolerance);
  EXPECT_NEAR(second.position.x, 1.0, kTolerance);
  EXPECT_NEAR(second.position.y, 2.0, kTolerance);
  EXPECT_NEAR(second.position.z, 3.0, kTolerance);
  EXPECT_NEAR(second.yaw, std::atan2(4.0, 3.0), kTolerance);
  EXPECT_NEAR(second.yaw_rate, 0.0, kTolerance);
  EXPECT_NEAR(std::hypot(second.velocity.x, second.velocity.y),
              0.0, kTolerance);
}

TEST(PlannerReferenceBuilder, ReenteringRotateModeRelatchesPosition) {
  PlannerReferenceBuilder builder;
  PlannerReferenceInput input;
  input.mode = ReferenceMode::kRotateInPlace;
  input.vehicle_x = 1.0;
  input.goal_x = 10.0;
  builder.Build(input);

  input.mode = ReferenceMode::kTrackPath;
  input.track_x = 5.0;
  builder.Build(input);

  input.mode = ReferenceMode::kRotateInPlace;
  input.vehicle_x = 7.0;
  const auto output = builder.Build(input);
  EXPECT_NEAR(output.position.x, 7.0, kTolerance);
}

TEST(PlannerReferenceBuilder, ManualReferenceIntegratesAConsistentTrajectory) {
  PlannerReferenceBuilder builder;
  PlannerReferenceInput input;
  input.mode = ReferenceMode::kManual;
  input.stamp = 1.0;
  input.vehicle_x = 1.0;
  input.vehicle_y = 2.0;
  input.vehicle_z = 3.0;
  input.manual_forward_velocity = 2.0;
  builder.Build(input);

  input.stamp = 1.1;
  const auto output = builder.Build(input);
  EXPECT_NEAR(output.position.x, 1.2, kTolerance);
  EXPECT_NEAR(output.position.y, 2.0, kTolerance);
  EXPECT_NEAR(output.velocity.x, 2.0, kTolerance);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
