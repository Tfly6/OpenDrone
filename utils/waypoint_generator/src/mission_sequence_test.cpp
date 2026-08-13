#include <gtest/gtest.h>

#include "waypoint_generator/mission_sequence.h"

namespace {

TEST(MissionSequenceTest, KeepsTheFirstWaypointActiveUntilExplicitAdvance) {
  waypoint_generator::MissionSequence sequence;
  sequence.reset(2);

  ASSERT_FALSE(sequence.complete());
  EXPECT_EQ(0u, sequence.activeIndex());
  const std::uint32_t first_command = sequence.commandSequence();
  EXPECT_NE(0u, first_command);
  EXPECT_EQ(first_command, sequence.commandSequence());

  ASSERT_TRUE(sequence.advance());
  EXPECT_EQ(1u, sequence.activeIndex());
  EXPECT_NE(first_command, sequence.commandSequence());
}

TEST(MissionSequenceTest, GivesAReplacementMissionANewCommandId) {
  waypoint_generator::MissionSequence sequence;
  sequence.reset(2);
  const std::uint32_t first_mission_command = sequence.commandSequence();

  sequence.reset(2);
  EXPECT_EQ(0u, sequence.activeIndex());
  EXPECT_NE(first_mission_command, sequence.commandSequence());
}

TEST(MissionSequenceTest, CompletesOnlyAfterTheLastWaypoint) {
  waypoint_generator::MissionSequence sequence;
  sequence.reset(2);

  ASSERT_TRUE(sequence.advance());
  EXPECT_FALSE(sequence.complete());
  EXPECT_FALSE(sequence.advance());
  EXPECT_TRUE(sequence.complete());
  EXPECT_EQ(0u, sequence.commandSequence());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
