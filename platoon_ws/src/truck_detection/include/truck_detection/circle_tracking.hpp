#ifndef TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
#define TRUCK_DETECTION__CIRCLE_TRACKING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

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

  // Parameters
  
  void obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr & msg);
};

} // namespace truck_detection

#endif // TRUCK_DETECTION__CIRCLE_TRACKING_HPP_
