#pragma once

#include <opendrone/PlannerOutputPoint.h>

namespace airfar_reference_bridge {

enum class ReferenceMode {
  kTrackPath,
  kRotateInPlace,
  kManual,
};

struct PlannerReferenceInput {
  ReferenceMode mode = ReferenceMode::kTrackPath;
  double stamp = 0.0;

  double vehicle_x = 0.0;
  double vehicle_y = 0.0;
  double vehicle_z = 0.0;
  double vehicle_yaw = 0.0;

  double track_x = 0.0;
  double track_y = 0.0;
  double track_z = 0.0;
  double track_yaw = 0.0;
  double path_speed = 0.0;
  double path_vertical_velocity = 0.0;
  double path_curvature = 0.0;
  double path_yaw_rate = 0.0;

  double goal_x = 0.0;
  double goal_y = 0.0;

  double manual_forward_velocity = 0.0;
  double manual_left_velocity = 0.0;
  double manual_vertical_velocity = 0.0;
  double manual_yaw_rate = 0.0;
};

// Converts path-follower intent into OpenDrone's world-frame kinematic
// contract. It deliberately does not translate the upstream roll/pitch or
// closed-loop yaw-rate commands: those are actuator-level feedback terms and
// would be applied a second time by an OpenDrone controller.
class PlannerReferenceBuilder {
 public:
  opendrone::PlannerOutputPoint Build(const PlannerReferenceInput& input);

 private:
  void LatchAtVehicle(const PlannerReferenceInput& input);

  bool initialized_ = false;
  ReferenceMode previous_mode_ = ReferenceMode::kTrackPath;
  double previous_stamp_ = 0.0;
  double latched_x_ = 0.0;
  double latched_y_ = 0.0;
  double latched_z_ = 0.0;
  double latched_yaw_ = 0.0;
};

}  // namespace airfar_reference_bridge
