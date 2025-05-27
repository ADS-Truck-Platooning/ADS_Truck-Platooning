#ifndef TRUCK_DETECTION__POINTCLOUD2_ROI_HPP_
#define TRUCK_DETECTION__POINTCLOUD2_ROI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace truck_detection
{

class PointCloud2ROI : public rclcpp::Node
{
public:
  explicit PointCloud2ROI(const rclcpp::NodeOptions & options);

private:
  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters (doubles for simplicity)
  double x_min_, x_max_;
  double y_min_, y_max_;
  double roi_angle_deg_;

  // Helpers
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void updateParameters();

  // Parameter event callback (optional, auto-update at runtime)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
    
} // namespace truck_detection

#endif // TRUCK_DETECTION__POINTCLOUD2_ROI_HPP_
