#include "truck_detection/circle_tracking.hpp"
#include <std_msgs/msg/bool.hpp>

namespace truck_detection
{

CircleTracking::CircleTracking(const rclcpp::NodeOptions & options)
: rclcpp::Node("circle_tracking", options)
{
  this->declare_parameter("truck_id", 0);
  this->declare_parameter("stop_distance", 5.0);
  updateParameters();

  const std::string in_topic = "/truck" + std::to_string(truck_id_) + "/raw_obstacles";
  sub_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
      in_topic, rclcpp::SensorDataQoS(),
      std::bind(&CircleTracking::obstaclesCallback, this, std::placeholders::_1));

  const std::string out_topic = "/truck" + std::to_string(truck_id_) + "/front_truck_pose";
  pub_ = this->create_publisher<geometry_msgs::msg::Pose>(out_topic, 10);

  emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
}

void CircleTracking::updateParameters()
{
  truck_id_ = this->get_parameter("truck_id").as_int();
  stop_distance_ = this->get_parameter("stop_distance").as_double();
}

void CircleTracking::obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr & msg)
{
  geometry_msgs::msg::Pose pose_msg;
  
  if (msg->circles.empty()) 
  {
    pose_msg.position.x = 0.0;
    pose_msg.position.y = 0.0;
    pose_msg.position.z = 0.0;
    pose_msg.orientation.w = 0.0;

    pub_->publish(pose_msg);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No circles detected – publishing NaN pose");
    return;
  }

  // 첫 번째 원을 초기값으로 설정
  const auto *closest_circle = &msg->circles.front();
  double min_dist_sq =
      closest_circle->center.x * closest_circle->center.x +
      closest_circle->center.y * closest_circle->center.y;

  // 나머지 원들과 비교해 가장 가까운 원 찾기
  for (const auto & circle : msg->circles) 
  {
    double dist_sq = circle.center.x * circle.center.x +
                     circle.center.y * circle.center.y;
    if (dist_sq < min_dist_sq) 
    {
      min_dist_sq = dist_sq;
      closest_circle = &circle;
    }
  }

  double x = closest_circle->center.x;
  double y = closest_circle->center.y;
  double distance = std::sqrt(min_dist_sq);

  RCLCPP_INFO(this->get_logger(),
              "Closest circle at (%.2f, %.2f) — distance %.2f m",
              x, y, distance);

  // 퍼블리시할 Pose 메시지 작성
  pose_msg.position.x = x;
  pose_msg.position.y = y;
  pose_msg.position.z = 0.0;
  pose_msg.orientation.w = 1.0;  // 단순 2D 위치이므로 단위 quaternion

  pub_->publish(pose_msg);

  if (truck_id_ == 0 && distance <= stop_distance_)
  {
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = true;
    emergency_pub_->publish(stop_msg);
  }
}

}
