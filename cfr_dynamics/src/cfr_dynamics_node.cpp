#include "cfr_dynamics/cfr_dynamics.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cfr_dynamics_ns::CfrDynamics>());
  rclcpp::shutdown();
  return 0;
}