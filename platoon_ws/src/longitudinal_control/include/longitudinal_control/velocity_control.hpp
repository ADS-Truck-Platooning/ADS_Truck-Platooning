#ifndef LONGITUDINAL_CONTROL__VELOCITY_CONTROL_HPP_
#define LONGITUDINAL_CONTROL__VELOCITY_CONTROL_HPP_

#include <algorithm>

namespace longitudinal_control
{
    
class VelocityController
{
public:
  VelocityController();
  VelocityController(double kp, double ki, double aw_gain, double throttle_limit);
  void param(double kp, double ki, double aw_gain, double throttle_limit);
  double update(double velocity_error, double dt);
  double update(double ref_vel, double meas_vel, double dt);

private:
  double kp_;
  double ki_;
  double aw_gain_;
  double throttle_limit_;
  double integral_;
};

} // namespace longitudinal_control

#endif // LONGITUDINAL_CONTROL__VELOCITY_CONTROL_HPP_
