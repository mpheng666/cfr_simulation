#include "path_tracer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTracer>());
  rclcpp::shutdown();
  return 0;
}