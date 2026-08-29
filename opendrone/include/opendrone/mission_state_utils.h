#pragma once

#include <cstdint>
#include <string>

#include <opendrone/MissionState.h>
#include <ros/ros.h>

namespace opendrone {

inline MissionState MakeMissionState(
    const std::string& frame_id, std::uint8_t mission_type, std::uint8_t status,
    std::uint32_t completed_items, std::uint32_t total_items,
    const std::string& detail = std::string()) {
  MissionState state;
  state.header.stamp = ros::Time::now();
  state.header.frame_id = frame_id;
  state.mission_type = mission_type;
  state.status = status;
  state.completed_items = completed_items;
  state.total_items = total_items;
  state.detail = detail;
  return state;
}

}  // namespace opendrone
