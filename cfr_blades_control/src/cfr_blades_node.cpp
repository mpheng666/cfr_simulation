#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CFRBladesControl : public rclcpp::Node
{
  public:
    CFRBladesControl()
    : Node("cfr_blades_node")
    {
        this->load_params();
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&CFRBladesControl::joy_callback, this, _1));
        blades_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);
        timer_ = this->create_wall_timer(
        200ms, std::bind(&CFRBladesControl::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto control_msg = std_msgs::msg::Float64MultiArray();
        control_msg.data.resize(N_BLADES);
        control_msg.data.front() = blades_speed_;
        control_msg.data.back() = -blades_speed_;
        // RCLCPP_INFO(this->get_logger(), "Blades speed: %f", control_msg.data.front() * 60.0);
        blades_publisher_->publish(control_msg);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if(msg->buttons.at(enable_button_))  blades_speed_ = std::clamp(msg->axes.at(speed_axis_) * max_speed_, 0.0, max_speed_); 
    }

    void load_params()
    {
        max_speed_ = this->declare_parameter("max_speed", 120.0) / 60.0;
        RCLCPP_INFO(this->get_logger(),"CFR blades running at maximum speed %f rpm", max_speed_ * 60.0);
        enable_button_ = this->declare_parameter("enable_button", 5);
        RCLCPP_INFO(this->get_logger(),"Enable blades button %i ", enable_button_);
        speed_axis_ = this->declare_parameter("speed_axis", 3);
        RCLCPP_INFO(this->get_logger(),"Blades speed axis %i ", speed_axis_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr blades_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const int N_BLADES = 2;
    double max_speed_ = 200.0 / 60.0;
    double blades_speed_;
    int enable_button_;
    int speed_axis_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CFRBladesControl>());
  rclcpp::shutdown();
  return 0;
}