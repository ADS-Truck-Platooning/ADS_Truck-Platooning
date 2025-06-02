#include "longitudinal_control/longitudinal_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<longitudinal_control::LongitudinalController>());
  rclcpp::shutdown();
  return 0;
}
