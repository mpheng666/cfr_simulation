#include "cfr_actuation/cfr_actuation.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cfr_actuation_ns::CfrActuation>());
  rclcpp::shutdown();
  return 0;
}