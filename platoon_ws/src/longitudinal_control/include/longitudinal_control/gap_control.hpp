#ifndef LONGITUDINAL_CONTROL__GAP_CONTROL_HPP_
#define LONGITUDINAL_CONTROL__GAP_CONTROL_HPP_

namespace longitudinal_control
{

class GapController
{
public:
  GapController();
  GapController(double kp, double kd, double desired_gap);
  void param(double kp, double kd, double desired_gap);
  double update(double current_gap, double gap_rate);
  double update(double ref_velocity, double current_gap, double gap_rate);

private:
  double kp_;
  double kd_;
  double desired_gap_;
};

} // namespace longitudinal_control

#endif // LONGITUDINAL_CONTROL__GAP_CONTROL_HPP_
