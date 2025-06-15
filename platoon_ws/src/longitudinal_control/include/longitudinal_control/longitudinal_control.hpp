#ifndef LONGITUDINAL_CONTROL__LONGITUDINAL_CONTROL_NODE_HPP_
#define LONGITUDINAL_CONTROL__LONGITUDINAL_CONTROL_NODE_HPP_

#include <longitudinal_control/gap_control.hpp>
#include <longitudinal_control/velocity_control.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"


namespace longitudinal_control
{

class LongitudinalController : public rclcpp::Node
{
public:
  explicit LongitudinalController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- Callbacks ------------------------------------------------------------
  void leadVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void refVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void leadPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void egoVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void egoPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void cameraOnCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void timerCallback();

  // --- Controllers ----------------------------------------------------------
  GapController     gap_ctrl_;
  VelocityController vel_ctrl_;

  double gap_kp_;
  double gap_kd_;
  double desired_gap_;

  double vel_kp_;
  double vel_ki_;
  double k_aw_;
  double throttle_limit_;
  double ff_gain_;

  double dec_rate_;

  // --- State variables ------------------------------------------------------
  double lead_x_;
  double lead_y_;
  double prev_lead_x_;
  double prev_lead_y_;
  double ego_x_;
  double current_gap_;
  double prev_gap_;
  double gap_rate_;
  double lead_velocity_;
  double ego_velocity_;
  double ref_velocity_;
  rclcpp::Time prev_time_;

  // --- ROS entities ---------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lead_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  sub_lead_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ego_vel_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ref_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_camera_on_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ref_vel_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_desired_gap_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  int truck_id_;
  double desired_velocity_;
  bool camera_on_{true};
};

} // namespace longitudinal_control

#endif // LONGITUDINAL_CONTROL__LONGITUDINAL_CONTROL_NODE_HPP_
