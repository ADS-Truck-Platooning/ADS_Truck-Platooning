#ifndef TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
#define TRUCK_DETECTION__CIRCLE_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "obstacle_detector/msg/obstacles.hpp"
#include "obstacle_detector/msg/circle_obstacle.hpp"
#include "obstacle_detector/msg/segment_obstacle.hpp"

namespace truck_detection
{

class CircleTracking : public rclcpp::Node
{
public:
  explicit CircleTracking(const rclcpp::NodeOptions & options);

private:
  // ROS I/O
  rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;

  // Parameters
  int truck_id_;
  double braking_decel_;
  double ego_velocity_;
  bool emergency_stop_;
  
  void obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr & msg);
  void velocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void updateParameters();
};

} // namespace truck_detection

#endif // TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
