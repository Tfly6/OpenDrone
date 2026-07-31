#include <airfar_reference_bridge/path_scale_schedule.h>

#include <algorithm>
#include <cmath>

namespace airfar_reference_bridge {
namespace {

constexpr double kScaleTolerance = 1e-9;

}  // namespace

std::vector<double> BuildPathScaleSchedule(
    const double configured_scale,
    const double minimum_scale,
    const double scale_step,
    const bool scale_by_speed,
    const double speed_ratio) {
  std::vector<double> scales;
  if (!std::isfinite(configured_scale) || !std::isfinite(minimum_scale) ||
      !std::isfinite(scale_step) || configured_scale <= 0.0 ||
      minimum_scale <= 0.0 || scale_step <= 0.0 ||
      minimum_scale > configured_scale) {
    return scales;
  }
  if (scale_by_speed && !std::isfinite(speed_ratio)) {
    return scales;
  }

  double initial_scale = configured_scale;
  if (scale_by_speed) {
    initial_scale *= std::max(0.0, std::min(1.0, speed_ratio));
  }
  initial_scale =
      std::max(minimum_scale, std::min(configured_scale, initial_scale));
  scales.push_back(initial_scale);

  double next_scale = initial_scale - scale_step;
  while (next_scale > minimum_scale + kScaleTolerance) {
    scales.push_back(next_scale);
    next_scale -= scale_step;
  }

  if (initial_scale > minimum_scale + kScaleTolerance) {
    scales.push_back(minimum_scale);
  }
  return scales;
}

}  // namespace airfar_reference_bridge
