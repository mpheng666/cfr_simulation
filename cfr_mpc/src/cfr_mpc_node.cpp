#include "cfr_mpc/cfr_mpc.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cfr_mpc::CFRMPC>());
  rclcpp::shutdown();
  return 0;
}