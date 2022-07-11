#include "cfr_states/cfr_states.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cfr_states_ns::CfrStates>());
  rclcpp::shutdown();
  return 0;
}