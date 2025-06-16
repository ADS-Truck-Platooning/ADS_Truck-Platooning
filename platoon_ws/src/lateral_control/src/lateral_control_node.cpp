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
  RCLCPP_INFO(this->get_logger(), "Truck ID: %d", truck_id_);

  this->declare_parameter<double>("look_ahead_distance", 20.0);
  look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
  RCLCPP_INFO(this->get_logger(), "Look Ahead Distance: %f", look_ahead_distance_);

  /* Subscribers */
  const std::string path_topic = "/platoon/truck" + std::to_string(truck_id_) + "/path";
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LateralControlNode::pathCallback, this, std::placeholders::_1));

  const std::string pose_topic = "/truck" + std::to_string(truck_id_) + "/front_truck_pose";
  front_truck_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      pose_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&LateralControlNode::poseCallback, this, std::placeholders::_1));

  const std::string camera_on_topic = "/truck" + std::to_string(truck_id_) + "/camera_on";
  camera_on_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      camera_on_topic,
      rclcpp::QoS(10),
      std::bind(&LateralControlNode::cameraOnCallback, this, std::placeholders::_1));

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

  std::optional<double> steering_rad = pp_->computeSteeringAngle(middle_points, {0.0, 0.0}, 0.0, truck_id_, lead_x_, lead_y_, camera_on_);
  prev_lead_x_ = lead_x_;
  prev_lead_y_ = lead_y_;

  double steering_deg;
  if (steering_rad) 
  {
    steering_deg = steering_rad.value() * 180.0 / M_PI;
    prev_steering_ = steering_deg;
    RCLCPP_INFO(this->get_logger(), "1");
  } 
  else 
  {
    steering_deg = prev_steering_;
    RCLCPP_INFO(this->get_logger(), "2");
  }

  publishSteering(steering_deg);
}

void LateralControlNode::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  lead_x_ = msg->position.x;
  lead_y_ = msg->position.y;

  if (lead_x_ == 0.0)
  {
    lead_x_ = prev_lead_x_;
    lead_y_ = prev_lead_y_;
  }
}

void LateralControlNode::cameraOnCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  camera_on_ = msg->data;
}

/* ───────── Publish Method ───────── */
void LateralControlNode::publishSteering(double steer_deg)
{
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(steer_deg);
  steer_pub_->publish(msg);
}

}  // namespace lateral_control
