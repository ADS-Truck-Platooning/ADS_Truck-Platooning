#ifndef LATERAL_CONTROL__PURE_PURSUIT_HPP_
#define LATERAL_CONTROL__PURE_PURSUIT_HPP_

#include <vector>
#include <utility>
#include <optional>

namespace lateral_control
{

class PurePursuit
{
public:
  PurePursuit(double look_ahead_dist = 20.0, double wheel_base = 2.0);

  std::optional<double> computeSteeringAngle(
      const std::vector<std::pair<double, double>>& path,
      const std::pair<double, double>& current_pos,
      double current_yaw,
      int truck_id,
      double lead_x,
      double lead_y,
      bool camera_on,
      double wheel_base      = -1.0,
      double look_ahead_dist = -1.0);

  std::optional<std::pair<double, double>> lastLookAheadPoint() const
  { return look_ahead_point_; }

private:
  static double normalizeAngle(double angle);
  static std::optional<std::size_t> getNearestIndex(
      const std::vector<std::pair<double,double>>& path,
      double x, double y);
  static std::optional<std::pair<double,double>> getLookAheadPoint(
      const std::vector<std::pair<double,double>>& path,
      double x, double y,
      double look_ahead_dist,
      int truck_id,
      double lead_x,
      double lead_y,
      bool camera_on);

  double look_ahead_dist_;
  double wheel_base_;

  std::optional<std::pair<double,double>> look_ahead_point_;
};

}  // namespace lateral_control

#endif  // LATERAL_CONTROL__PURE_PURSUIT_HPP_
