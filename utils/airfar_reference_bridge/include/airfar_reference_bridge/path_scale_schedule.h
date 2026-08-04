#pragma once

#include <vector>

namespace airfar_reference_bridge {

// Builds the collision-horizon scales to try, from longest to shortest.
//
// Speed scaling may produce a value that is not aligned with scale_step. The
// exact minimum_scale is therefore appended explicitly instead of relying on
// repeated subtraction to happen to land on it. An empty result denotes an
// invalid configuration.
std::vector<double> BuildPathScaleSchedule(
    double configured_scale,
    double minimum_scale,
    double scale_step,
    bool scale_by_speed,
    double speed_ratio);

}  // namespace airfar_reference_bridge
