#include "longitudinal_control/longitudinal_control.hpp"
#include <chrono>

namespace longitudinal_control
{

LongitudinalController::LongitudinalController(const rclcpp::NodeOptions & options)
: rclcpp::Node("longitudinal_controller", options),
  lead_x_(0.0),
  lead_y_(0.0),
  prev_lead_x_(0.0),
  prev_lead_y_(0.0),
  ego_x_(0.0),
  current_gap_(0.0),
  prev_gap_(0.0),
  gap_rate_(0.0),
  lead_velocity_(0.0),
  ego_velocity_(0.0),
  ref_velocity_(0.0),
  prev_time_(this->get_clock()->now()),
  truck_id_(0),
  desired_velocity_(0.0)
{
    this->declare_parameter("gap_kp", 0.8);
    this->declare_parameter("gap_kd", 0.4);
    this->declare_parameter("desired_gap", 2.0);
    gap_kp_ = this->get_parameter("gap_kp").as_double();
    gap_kd_ = this->get_parameter("gap_kd").as_double();
    desired_gap_ = this->get_parameter("desired_gap").as_double();

    this->declare_parameter("vel_kp", 1.0);
    this->declare_parameter("vel_ki", 2.0);
    this->declare_parameter("k_aw", 1e-4);
    this->declare_parameter("throttle_limit", 1.0);
    this->declare_parameter("ff_gain", 0.05);
    vel_kp_ = this->get_parameter("vel_kp").as_double();
    vel_ki_ = this->get_parameter("vel_ki").as_double();
    k_aw_ = this->get_parameter("k_aw").as_double();
    throttle_limit_ = this->get_parameter("throttle_limit").as_double();
    ff_gain_ = this->get_parameter("ff_gain").as_double();

    this->declare_parameter("truck_id", 0);
    this->declare_parameter("desired_velocity", 36.0);
    truck_id_ = this->get_parameter("truck_id").as_int();
    desired_velocity_ = this->get_parameter("desired_velocity").as_double();

    gap_ctrl_.param(gap_kp_, gap_kd_, desired_gap_);
    vel_ctrl_.param(vel_kp_, vel_ki_, k_aw_, throttle_limit_, ff_gain_);

    // Subscriptions -----------------------------------------------------------
    if (truck_id_ != 0)
    {
      const std::string lead_vel_topic_ = "/truck" + std::to_string(truck_id_-1) + "/velocity";
      sub_lead_vel_ = create_subscription<std_msgs::msg::Float32>(
      lead_vel_topic_, 10,
      std::bind(&LongitudinalController::leadVelocityCallback, this, std::placeholders::_1));

      const std::string lead_pose_topic_ = "/truck" + std::to_string(truck_id_) + "/front_truck_pose";
      sub_lead_pose_ = create_subscription<geometry_msgs::msg::Pose>(
      lead_pose_topic_, 10,
      std::bind(&LongitudinalController::leadPoseCallback, this, std::placeholders::_1));

      const std::string ref_vel_topic_ = "/truck" + std::to_string(truck_id_-1) + "/reference_velocity";
      sub_ref_vel_ = create_subscription<std_msgs::msg::Float32>(
      ref_vel_topic_, 10,
      std::bind(&LongitudinalController::refVelocityCallback, this, std::placeholders::_1));
    }
    
    const std::string ego_vel_topic_ = "/truck" + std::to_string(truck_id_) + "/velocity";
    sub_ego_vel_ = create_subscription<std_msgs::msg::Float32>(
    ego_vel_topic_, 10,
    std::bind(&LongitudinalController::egoVelocityCallback, this, std::placeholders::_1));

    // Publisher ---------------------------------------------------------------
    const std::string throttle_topic_ = "/truck" + std::to_string(truck_id_) + "/throttle_control";
    pub_throttle_ = create_publisher<std_msgs::msg::Float32>(
    throttle_topic_, 10);

    if (truck_id_ != 2)
    {
      const std::string ref_vel_topic_ = "/truck" + std::to_string(truck_id_) + "/reference_velocity";
      pub_ref_vel_ = create_publisher<std_msgs::msg::Float32>(
      ref_vel_topic_, 10);
    }

    // Control loop timer (50 Hz) ---------------------------------------------
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&LongitudinalController::timerCallback, this));
}

// --- Callbacks --------------------------------------------------------------
void LongitudinalController::leadVelocityCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  lead_velocity_ = msg->data;
}

void LongitudinalController::leadPoseCallback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  lead_x_ = msg->position.x;
  lead_y_ = msg->position.y;

  if (lead_x_ == 0.0)
  {
    lead_x_ = prev_lead_x_;
    lead_y_ = prev_lead_y_;
  }
}

void LongitudinalController::refVelocityCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  ref_velocity_ = msg->data;
}

void LongitudinalController::egoVelocityCallback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  ego_velocity_ = msg->data;
}

void LongitudinalController::timerCallback()
{
  // --- Time step -----------------------------------------------------------
  const rclcpp::Time now = this->get_clock()->now();
  const double dt = (now - prev_time_).seconds();
  prev_time_ = now;
  if (dt <= 0.0) return;  // protect against clock issues

  // --- Gap & rate ----------------------------------------------------------
  current_gap_ = std::sqrt(lead_x_ * lead_x_ + lead_y_ * lead_y_);
  prev_lead_x_ = lead_x_;
  prev_lead_y_ = lead_y_;
  gap_rate_ = (current_gap_ - prev_gap_) / dt;
  prev_gap_ = current_gap_;

  // --- Layer 1: PD gap -> desired velocity --------------------------------
  if (truck_id_ != 0)
  {
    // const double vel_correction = gap_ctrl_.update(current_gap_, gap_rate_);
    // desired_velocity_ = lead_velocity_ + vel_correction;
    const double vel_correction = gap_ctrl_.update(ref_velocity_, current_gap_, gap_rate_);
    desired_velocity_ = vel_correction;
    RCLCPP_INFO(this->get_logger(), "desired_velocity: %f", desired_velocity_);
  }

  if (truck_id_ != 2)
  {
    std_msgs::msg::Float32 msg;
    msg.data = desired_velocity_;
    pub_ref_vel_->publish(msg);
  }

  // --- Layer 2: PI velocity -> throttle -----------------------------------
  double throttle_u;
  if (truck_id_ == 0)
  {
    // const double vel_error = std::clamp(desired_velocity_ - ego_velocity_, -2.0, 2.0);
    const double vel_error = desired_velocity_ - ego_velocity_;
    throttle_u  = vel_ctrl_.update(vel_error, dt);
  }
  else
  {
    throttle_u  = vel_ctrl_.update(desired_velocity_, ego_velocity_, dt);
  }  
  const double throttle_cmd= std::clamp(throttle_u, 0.0, throttle_limit_);

  std_msgs::msg::Float32 msg;
  msg.data = throttle_cmd;
  pub_throttle_->publish(msg);
}

}
