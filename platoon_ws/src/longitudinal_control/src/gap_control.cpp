#include "longitudinal_control/gap_control.hpp"

namespace longitudinal_control
{

GapController::GapController() {}
    
GapController::GapController(double kp, double kd, double desired_gap)
: kp_(kp), kd_(kd), desired_gap_(desired_gap) {}

void GapController::param(double kp, double kd, double desired_gap)
{
  kp_ = kp;
  kd_ = kd;
  desired_gap_ = desired_gap;
}

double GapController::update(double current_gap, double gap_rate)
{
  const double error      = current_gap - desired_gap_;   // d_des − d
  const double vel_corr   = kp_ * error - kd_ * gap_rate; // PD
  return vel_corr;
}

double GapController::update(double ref_velocity, double current_gap, double gap_rate)
{
  const double gap_error      = desired_gap_ - current_gap;   // d_des − d
  const double vel_corr   = ref_velocity - kp_ * gap_error - kd_ * gap_rate; // PD
  return vel_corr;
}

}
