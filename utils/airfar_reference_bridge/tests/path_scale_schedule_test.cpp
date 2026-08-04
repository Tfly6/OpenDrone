#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <airfar_reference_bridge/path_scale_schedule.h>

namespace {

using airfar_reference_bridge::BuildPathScaleSchedule;

constexpr double kTolerance = 1e-9;

void ExpectScales(const std::vector<double>& actual,
                  const std::vector<double>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], kTolerance);
  }
}

TEST(PathScaleSchedule, AlignedFallbackEndsAtExactMinimum) {
  ExpectScales(BuildPathScaleSchedule(2.0, 0.5, 0.5, false, 1.0),
               {2.0, 1.5, 1.0, 0.5});
}

TEST(PathScaleSchedule, UnalignedSpeedScaleStillTriesExactMinimum) {
  ExpectScales(BuildPathScaleSchedule(2.0, 0.5, 0.25, true, 1.0 / 3.0),
               {2.0 / 3.0, 0.5});
}

TEST(PathScaleSchedule, SpeedBelowMinimumClampsToSingleMinimumAttempt) {
  ExpectScales(BuildPathScaleSchedule(2.0, 0.5, 0.25, true, 0.1), {0.5});
}

TEST(PathScaleSchedule, SpeedRatioIsClampedToUnitInterval) {
  ExpectScales(BuildPathScaleSchedule(2.0, 0.5, 0.5, true, 2.0),
               {2.0, 1.5, 1.0, 0.5});
}

TEST(PathScaleSchedule, InvalidConfigurationIsRejected) {
  EXPECT_TRUE(BuildPathScaleSchedule(0.0, 0.5, 0.25, false, 1.0).empty());
  EXPECT_TRUE(BuildPathScaleSchedule(0.5, 1.0, 0.25, false, 1.0).empty());
  EXPECT_TRUE(BuildPathScaleSchedule(2.0, 0.5, 0.0, false, 1.0).empty());
  EXPECT_TRUE(BuildPathScaleSchedule(
                  2.0, 0.5, 0.25, true,
                  std::numeric_limits<double>::quiet_NaN())
                  .empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
