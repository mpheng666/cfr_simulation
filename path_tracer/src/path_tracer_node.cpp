#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PathTracer : public rclcpp::Node
{
  public:
    PathTracer()
    : Node("path_tracer")
    {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PathTracer::odom_callback, this, _1));
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("cfr_path", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&PathTracer::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto pose_stamped = geometry_msgs::msg::PoseStamped();
        message.header = odom_.header;
        pose_stamped.pose = odom_.pose.pose;
        message.poses.emplace_back(pose_stamped);
        path_publisher_->publish(message);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_.header = msg->header;
        odom_.pose = msg->pose;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Path message = nav_msgs::msg::Path();

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTracer>());
  rclcpp::shutdown();
  return 0;
}