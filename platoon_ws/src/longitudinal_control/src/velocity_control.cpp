#include "longitudinal_control/velocity_control.hpp"

namespace longitudinal_control
{

VelocityController::VelocityController() {}

VelocityController::VelocityController(double kp, double ki,
                                       double aw_gain, double throttle_limit)
: kp_(kp), ki_(ki), aw_gain_(aw_gain),
  throttle_limit_(throttle_limit), integral_(0.0) {}

void VelocityController::param(double kp, double ki, double aw_gain, double throttle_limit)
{
    kp_ = kp;
    ki_ = ki;
    aw_gain_ = aw_gain;
    throttle_limit_ = throttle_limit;
}

double VelocityController::update(double velocity_error, double dt)
{
  integral_ += velocity_error * dt;
  double u = kp_ * velocity_error + ki_ * integral_;

  // --- saturation + anti‑windup -------------------------------------------
  if (u > throttle_limit_) {
    integral_ -= aw_gain_ * (u - throttle_limit_);
    u = throttle_limit_;
  } else if (u < 0.0) {
    // 브레이크는 여기서 처리하지 않고, 단순히 스로틀 0 으로 클램프
    integral_ -= aw_gain_ * u;  // 음의 포화에 대한 급격한 적분 감소
    u = 0.0;
  }
  return u;
}

double VelocityController::update(double ref_vel,
                                  double meas_vel,
                                  double dt)
{
  // --- Feed-Forward --------------------------------------------------
  double u_ff = 1.0 * ref_vel;               // 가장 단순한 형태 (선형 비례)

  // --- PI ------------------------------------------------------------
  double vel_error = ref_vel - meas_vel;
  integral_ += vel_error * dt;
  double u_pi = kp_ * vel_error + ki_ * integral_;

  double u = u_ff + u_pi;                    // FF + PI 합산

  // --- saturation + back-calc AW ------------------------------------
  if (u > throttle_limit_) {
    integral_ -= aw_gain_ * (u - throttle_limit_);
    u = throttle_limit_;
  } else if (u < 0.0) {
    integral_ -= aw_gain_ * u;
    u = 0.0;
  }
  return u;
}

}
