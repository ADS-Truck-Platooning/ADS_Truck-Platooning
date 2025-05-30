#include "truck_detection/pointcloud2_roi.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <cmath>

#include <visualization_msgs/msg/marker.hpp>

namespace truck_detection
{

PointCloud2ROI::PointCloud2ROI(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud2_roi", options)
{
  // Declare parameters with defaults
  this->declare_parameter("truck_id", 0);
  this->declare_parameter("x_min", 0.0);
  this->declare_parameter("x_max", 30.0);
  this->declare_parameter("y_min", -5.0);
  this->declare_parameter("y_max",  5.0);
  this->declare_parameter("roi_angle_deg", 45.0);

  updateParameters();   // cache first set

  const std::string in_topic = "/truck" + std::to_string(truck_id_) + "/front_lidar";
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2ROI::cloudCallback, this, std::placeholders::_1));
  
  const std::string out_topic = "/truck" + std::to_string(truck_id_) + "/front_lidar/roi";
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, 10);

  const std::string marker_topic = "/truck" + std::to_string(truck_id_) + "/front_lidar/roi_marker";
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 1);
}

void PointCloud2ROI::updateParameters()
{
  truck_id_ = this->get_parameter("truck_id").as_int();
  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  roi_angle_deg_ = this->get_parameter("roi_angle_deg").as_double();
}

void PointCloud2ROI::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ> pcl_in;
  pcl::fromROSMsg(*msg, pcl_in);

  pcl::PointCloud<pcl::PointXYZ> pcl_out;
  pcl_out.header = pcl_in.header;

  double angle_limit = roi_angle_deg_ * M_PI / 180.0;

  for (const auto & pt : pcl_in.points)
  {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;

    // X-Y ROI
    if (pt.x < x_min_ || pt.x > x_max_) continue;
    if (pt.y < y_min_ || pt.y > y_max_) continue;

    // Forward angle ROI (symmetric about +X)
    double ang = std::atan2(pt.y, pt.x);
    if (std::fabs(ang) > angle_limit) continue;

    pcl_out.points.emplace_back(pt);
  }

  // Publish if not empty
  if (!pcl_out.points.empty())
  {
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(pcl_out, out_msg);
    pub_->publish(out_msg);
  }

  // visualization
  visualization_msgs::msg::Marker mk;
  mk.header = msg->header;
  mk.ns     = "roi";
  mk.id     = 0;
  mk.type   = visualization_msgs::msg::Marker::LINE_LIST;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.scale.x = 0.05;                 // 선 굵기 [m]
  mk.color.r = 0.0f; mk.color.g = 1.0f; mk.color.b = 0.0f; mk.color.a = 1.0f;

  /* ── 사각형 경계 ── */
  auto add_line = [&mk](float x1, float y1, float x2, float y2)
  {
    geometry_msgs::msg::Point p1, p2;
    p1.x = x1; p1.y = y1; p1.z = 0.0;
    p2.x = x2; p2.y = y2; p2.z = 0.0;
    mk.points.push_back(p1); mk.points.push_back(p2);
  };
  add_line(x_min_, y_min_, x_max_, y_min_);
  add_line(x_max_, y_min_, x_max_, y_max_);
  add_line(x_max_, y_max_, x_min_, y_max_);
  add_line(x_min_, y_max_, x_min_, y_min_);

  /* ── 각도 제한선(쐐기형) ── */
  float xa = x_max_;
  float ya = std::tan( angle_limit) * xa;
  float yb = std::tan(-angle_limit) * xa;
  if (ya >  y_max_) { xa =  y_max_ / std::tan( angle_limit); ya =  y_max_; }
  if (yb <  y_min_) { xa = (y_min_) / std::tan(-angle_limit); yb =  y_min_; }

  add_line(0.0, 0.0, xa, ya);   // +각도
  add_line(0.0, 0.0, xa, yb);   // -각도

  marker_pub_->publish(mk);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(truck_detection::PointCloud2ROI)
