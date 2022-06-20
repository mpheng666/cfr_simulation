#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyPublisher : public rclcpp::Node
{
  public:
    JoyPublisher()
    : Node("joy_publisher_node")
    {
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyPublisher::joy_callback, this, _1));
        joy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cfr_joy", 10);
        timer_ = this->create_wall_timer(
        200ms, std::bind(&JoyPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {

    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        double linear_x_joy = msg->axes.at(3);
        double angular_z_joy = msg->axes.at(2);
        double linear_y_joy = msg->axes.at(0);
        double blade_control_joy = msg->axes.at(1);

        auto pub_msg = std_msgs::msg::Float64MultiArray();
        pub_msg.data.resize(4);
        pub_msg.data.at(0) = linear_x_joy;
        pub_msg.data.at(1) = angular_z_joy;
        pub_msg.data.at(2) = linear_y_joy;
        pub_msg.data.at(3) = blade_control_joy;
        joy_publisher_->publish(pub_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joy_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyPublisher>());
  rclcpp::shutdown();
  return 0;
}