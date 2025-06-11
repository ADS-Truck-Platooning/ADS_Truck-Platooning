#ifndef TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
#define TRUCK_DETECTION__CIRCLE_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

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
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;

  // Parameters
  int truck_id_;
  double stop_distance_;
  
  void obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr & msg);
  void updateParameters();
};

} // namespace truck_detection

#endif // TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
