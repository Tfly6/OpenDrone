#include <airfar_reference_bridge/planner_reference.h>

#include <algorithm>
#include <cmath>

namespace airfar_reference_bridge {
namespace {

constexpr double kSmallDistance = 1e-6;
constexpr double kMaximumManualIntegrationStep = 0.2;

uint32_t ReferenceMask() {
  return opendrone::PlannerOutputPoint::VALID_POSITION |
         opendrone::PlannerOutputPoint::VALID_VELOCITY |
         opendrone::PlannerOutputPoint::VALID_ACCELERATION |
         opendrone::PlannerOutputPoint::VALID_YAW |
         opendrone::PlannerOutputPoint::VALID_YAW_RATE;
}

}  // namespace

void PlannerReferenceBuilder::LatchAtVehicle(
    const PlannerReferenceInput& input) {
  latched_x_ = input.vehicle_x;
  latched_y_ = input.vehicle_y;
  latched_z_ = input.vehicle_z;
  latched_yaw_ = input.vehicle_yaw;
}

opendrone::PlannerOutputPoint PlannerReferenceBuilder::Build(
    const PlannerReferenceInput& input) {
  opendrone::PlannerOutputPoint output;
  output.valid_mask = ReferenceMask();

  const bool mode_changed = !initialized_ || input.mode != previous_mode_;
  if (mode_changed &&
      (input.mode == ReferenceMode::kRotateInPlace ||
       input.mode == ReferenceMode::kManual)) {
    LatchAtVehicle(input);
  }

  if (input.mode == ReferenceMode::kTrackPath) {
    output.position.x = input.track_x;
    output.position.y = input.track_y;
    output.position.z = input.track_z;
    output.velocity.x = input.path_speed * std::cos(input.track_yaw);
    output.velocity.y = input.path_speed * std::sin(input.track_yaw);
    output.velocity.z = input.path_vertical_velocity;
    output.acceleration.x =
        -input.path_curvature * input.path_speed * input.path_speed *
        std::sin(input.track_yaw);
    output.acceleration.y =
        input.path_curvature * input.path_speed * input.path_speed *
        std::cos(input.track_yaw);
    output.acceleration.z = 0.0;
    output.yaw = input.track_yaw;
    output.yaw_rate = input.path_yaw_rate;
  } else if (input.mode == ReferenceMode::kRotateInPlace) {
    output.position.x = latched_x_;
    output.position.y = latched_y_;
    output.position.z = latched_z_;
    const double goal_dx = input.goal_x - latched_x_;
    const double goal_dy = input.goal_y - latched_y_;
    if (std::hypot(goal_dx, goal_dy) > kSmallDistance) {
      latched_yaw_ = std::atan2(goal_dy, goal_dx);
    }
    output.yaw = latched_yaw_;
    output.yaw_rate = 0.0;
  } else {
    double dt = 0.0;
    if (!mode_changed && input.stamp > previous_stamp_) {
      dt = std::min(input.stamp - previous_stamp_,
                    kMaximumManualIntegrationStep);
    }
    latched_yaw_ += input.manual_yaw_rate * dt;
    const double velocity_x =
        input.manual_forward_velocity * std::cos(latched_yaw_) -
        input.manual_left_velocity * std::sin(latched_yaw_);
    const double velocity_y =
        input.manual_forward_velocity * std::sin(latched_yaw_) +
        input.manual_left_velocity * std::cos(latched_yaw_);
    latched_x_ += velocity_x * dt;
    latched_y_ += velocity_y * dt;
    latched_z_ += input.manual_vertical_velocity * dt;

    output.position.x = latched_x_;
    output.position.y = latched_y_;
    output.position.z = latched_z_;
    output.velocity.x = velocity_x;
    output.velocity.y = velocity_y;
    output.velocity.z = input.manual_vertical_velocity;
    output.yaw = latched_yaw_;
    output.yaw_rate = input.manual_yaw_rate;
  }

  previous_mode_ = input.mode;
  previous_stamp_ = input.stamp;
  initialized_ = true;
  return output;
}

}  // namespace airfar_reference_bridge
