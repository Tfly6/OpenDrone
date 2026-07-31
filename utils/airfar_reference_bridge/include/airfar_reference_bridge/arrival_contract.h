#pragma once

#include <cmath>

namespace airfar_reference_bridge {

inline double GoalDistance3D(double x, double y, double z,
                             double goal_x, double goal_y, double goal_z) {
  return std::sqrt(
      (goal_x - x) * (goal_x - x) +
      (goal_y - y) * (goal_y - y) +
      (goal_z - z) * (goal_z - z));
}

inline bool IsWithinGoalTolerance(double x, double y, double z,
                                  double goal_x, double goal_y, double goal_z,
                                  double tolerance) {
  return tolerance > 0.0 &&
         GoalDistance3D(x, y, z, goal_x, goal_y, goal_z) <= tolerance;
}

}  // namespace airfar_reference_bridge
