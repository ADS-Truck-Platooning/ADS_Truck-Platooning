#include "truck_detection/circle_tracking.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<truck_detection::CircleTracking>(rclcpp::NodeOptions{});
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
