#ifndef WAYPOINT_GENERATOR_MISSION_SEQUENCE_H
#define WAYPOINT_GENERATOR_MISSION_SEQUENCE_H

#include <cstddef>
#include <cstdint>

namespace waypoint_generator {

// Keeps an ordered mission independent from its geometry.  The command id is
// stable while a waypoint is being re-published and changes for every new leg.
class MissionSequence {
 public:
  void reset(std::size_t waypoint_count) {
    waypoint_count_ = waypoint_count;
    active_index_ = 0;
    complete_ = waypoint_count == 0;
    command_sequence_ = complete_ ? 0 : nextCommandSequence();
  }

  bool advance() {
    if (complete_) {
      return false;
    }
    ++active_index_;
    if (active_index_ >= waypoint_count_) {
      complete_ = true;
      command_sequence_ = 0;
      return false;
    }
    command_sequence_ = nextCommandSequence();
    return true;
  }

  std::size_t activeIndex() const { return active_index_; }
  std::size_t waypointCount() const { return waypoint_count_; }
  bool complete() const { return complete_; }
  std::uint32_t commandSequence() const { return command_sequence_; }

 private:
  std::uint32_t nextCommandSequence() {
    ++next_command_sequence_;
    // Header.seq == 0 is reserved for publishers that do not implement this
    // ordered-mission protocol.
    if (next_command_sequence_ == 0) {
      ++next_command_sequence_;
    }
    return next_command_sequence_;
  }

  std::size_t waypoint_count_ = 0;
  std::size_t active_index_ = 0;
  bool complete_ = true;
  std::uint32_t next_command_sequence_ = 0;
  std::uint32_t command_sequence_ = 0;
};

}  // namespace waypoint_generator

#endif  // WAYPOINT_GENERATOR_MISSION_SEQUENCE_H
