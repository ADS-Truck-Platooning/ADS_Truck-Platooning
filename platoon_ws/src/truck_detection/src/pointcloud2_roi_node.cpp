#include "truck_detection/pointcloud2_roi.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<truck_detection::PointCloud2ROI>(rclcpp::NodeOptions{});
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
