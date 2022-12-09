#include "cfr_kinematics_states/cfr_kinematics_states.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cfr_kinemtics_states_ns::CfrKinematicsStates>());
  rclcpp::shutdown();
  return 0;
}