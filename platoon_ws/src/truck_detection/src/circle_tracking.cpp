#include "truck_detection/circle_tracking.hpp"

namespace truck_detection
{

CircleTracking::CircleTracking(const rclcpp::NodeOptions & options)
: rclcpp::Node("circle_tracking", options)
{
  sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
      "raw_obstacles", rclcpp::SensorDataQoS(),
      std::bind(&CircleTracking::obstaclesCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<geometry_msgs::msg::Pose>("circles", 10);
}

void CircleTracking::obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr & msg)
{
  for (const auto & circle : msg->circles)
  {
    double x = circle.center.x;
    double y = circle.center.y;
    RCLCPP_INFO(this->get_logger(), "Detected circle at (%.2f, %.2f)", x, y);
  }
}

}
