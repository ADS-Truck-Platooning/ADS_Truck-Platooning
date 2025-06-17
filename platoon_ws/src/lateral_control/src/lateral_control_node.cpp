#include "lateral_control/lateral_control_node.hpp"

#include <cmath>
#include <string>

namespace lateral_control
{

LateralControlNode::LateralControlNode()
: Node("lateral_control_node")
{
  /* Parameters */
  this->declare_parameter<int>("truck_id", 0);
  truck_id_ = this->get_parameter("truck_id").as_int();
  // RCLCPP_INFO(this->get_logger(), "Truck ID: %d", truck_id_);

  this->declare_parameter<double>("look_ahead_distance", 20.0);
  look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
  // RCLCPP_INFO(this->get_logger(), "Look Ahead Distance: %f", look_ahead_distance_);

  /* Subscribers */
  const std::string path_topic = "/platoon/truck" + std::to_string(truck_id_) + "/path";
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LateralControlNode::pathCallback, this, std::placeholders::_1));

  /* Publishers */
  const std::string steer_topic = "/truck" + std::to_string(truck_id_) + "/steer_control";
  steer_pub_ = this->create_publisher<std_msgs::msg::Float32>(steer_topic, 10);

  /* Pure Pursuit Initialization */
  pp_ = std::make_unique<PurePursuit>(look_ahead_distance_, 2.0);
}

/* ───────── Path Callback ───────── */
void LateralControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::vector<std::pair<double, double>> middle_points;
  middle_points.reserve(msg->poses.size());

  for (const auto & pose_stamped : msg->poses) 
    middle_points.emplace_back(pose_stamped.pose.position.x, pose_stamped.pose.position.y);

  std::optional<double> steering_rad = pp_->computeSteeringAngle(middle_points, {0.0, 0.0}, 0.0);

  double steering_deg;
  if (steering_rad) 
  {
    steering_deg = steering_rad.value() * 180.0 / M_PI;
    prev_steering_ = steering_deg;
  } 
  else 
  {
    steering_deg = prev_steering_;
  }

  publishSteering(steering_deg);
}

/* ───────── Publish Method ───────── */
void LateralControlNode::publishSteering(double steer_deg)
{
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(steer_deg);
  steer_pub_->publish(msg);
}

}  // namespace lateral_control
