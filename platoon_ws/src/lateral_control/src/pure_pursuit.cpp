#include "lateral_control/pure_pursuit.hpp"
#include <cmath>
#include <limits>

namespace lateral_control
{

PurePursuit::PurePursuit(double look_ahead_dist, double wheel_base)
: look_ahead_dist_(look_ahead_dist),
  wheel_base_(wheel_base)
{}

double PurePursuit::normalizeAngle(double angle)
{
  while (angle >= M_PI)  angle -= 2.0 * M_PI;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::optional<std::size_t> PurePursuit::getNearestIndex(
    const std::vector<std::pair<double,double>>& path,
    double x, double y)
{
  if (path.empty()) return std::nullopt;

  double min_dist = std::numeric_limits<double>::max();
  std::size_t min_idx = 0;

  for (std::size_t i = 0; i < path.size(); i++) {
    double dx = x - path[i].first;
    double dy = y - path[i].second;
    double dist_sq = dx*dx + dy*dy;
    if (dist_sq < min_dist) {
      min_dist = dist_sq;
      min_idx = i;
    }
  }
  return min_idx;
}

std::optional<std::pair<double,double>> PurePursuit::getLookAheadPoint(
    const std::vector<std::pair<double,double>>& path,
    double x, double y,
    double look_ahead_dist)
{
  if (path.empty()) return std::nullopt;

  for (const auto& pt : path) 
  {
    double dx = pt.first  - x;
    double dy = pt.second - y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist >= look_ahead_dist) 
    {
      return pt;
    }
  }
  return path[path.size()-1];
}

std::optional<double> PurePursuit::computeSteeringAngle(
    const std::vector<std::pair<double, double>>& path,
    const std::pair<double, double>& current_pos,
    double current_yaw,
    double look_ahead_dist,
    double wheel_base)
{
  if (path.empty()) {
    look_ahead_point_.reset();
    return std::nullopt;
  }

  if (look_ahead_dist < 0.0) look_ahead_dist = look_ahead_dist_;
  if (wheel_base      < 0.0) wheel_base      = wheel_base_;

  const auto lap = getLookAheadPoint(path,
                                     current_pos.first,
                                     current_pos.second,
                                     look_ahead_dist);

  if (!lap) 
  {
    look_ahead_point_.reset();
    return std::nullopt;
  }

  look_ahead_point_ = lap;

  double lx = lap->first;
  double ly = lap->second;

  double angle_to_target = std::atan2(ly - current_pos.second,
                                      lx - current_pos.first);

  double heading_error = normalizeAngle(angle_to_target - current_yaw);

  double steering_angle = std::atan2(
      2.0 * wheel_base * std::sin(heading_error),
      look_ahead_dist);

  return steering_angle;
}

}  // namespace lateral_control
