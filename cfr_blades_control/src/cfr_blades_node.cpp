#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <locale>
#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CFRBladesControl : public rclcpp::Node {
public:
    CFRBladesControl()
        : Node("cfr_blades_node")
    {
        this->load_params();
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&CFRBladesControl::joy_callback, this, _1));
        blade_speed_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "cfr_blade_speed", 10,
        std::bind(&CFRBladesControl::blade_speed_callback, this, _1));
        blades_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "forward_velocity_controller/commands", 10);
        start_publisher_ = this->create_publisher<std_msgs::msg::Bool>("allow_move", 10);
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
        // RCLCPP_INFO(this->get_logger(), "Blades speed: %f", control_msg.data.front()
        // * 60.0);
        if (abs(blades_speed_) > 0.1) {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            start_publisher_->publish(msg);
        }
        else {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            // msg.data = false;
            start_publisher_->publish(msg);
        }
        blades_publisher_->publish(control_msg);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // [[maybe_unused]] auto temp = (msg->axes.at(speed_axis_) - joy_offset_);
        // // RCLCPP_INFO(this->get_logger(), "temp %f", temp);
        // if (is_start_) {
        //     if (msg->buttons.at(enable_button_) || !require_enable_button_) {
        //         // blades_speed_ = std::clamp(msg->axes.at(speed_axis_) * max_speed_,
        //         0.0,
        //         // max_speed_);
        //         blades_speed_ = (msg->axes.at(speed_axis_) - joy_offset_) * max_speed_;
        //     }
        // }

        // else {
        //     if (msg->buttons.at(start_button_) > 0) {
        //         is_start_ = true;
        //     }

        //     if (abs(msg->axes.at(speed_axis_) - joy_offset_) > joy_deadzone_) {
        //         // system("notify-send -u low --hint int:transient:1 'CFR SIMULATION'
        //         // 'Please start the engine!'");
        //     }
        // }
    }

    void blade_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        blades_speed_ = msg->data;
        std::cout << "blade speed: " << blades_speed_;
    }

    void load_params()
    {
        max_speed_ = this->declare_parameter("max_speed", 120.0) / 60.0;
        RCLCPP_INFO(this->get_logger(), "CFR blades running at maximum speed %f rpm",
                    max_speed_ * 60.0);
        enable_button_ = this->declare_parameter("enable_button", 0);
        RCLCPP_INFO(this->get_logger(), "Enable blades button %i ", enable_button_);
        start_button_ = this->declare_parameter("start_button", 0);
        RCLCPP_INFO(this->get_logger(), "Start button %i ", start_button_);
        speed_axis_ = this->declare_parameter("speed_axis", 3);
        RCLCPP_INFO(this->get_logger(), "Blades speed axis %i ", speed_axis_);
        require_enable_button_ = this->declare_parameter("require_enable_button", false);
        RCLCPP_INFO(this->get_logger(), "Require enable button is set to %d",
                    require_enable_button_);
        joy_offset_ = this->declare_parameter("joy_offset", 0.0);
        RCLCPP_INFO(this->get_logger(), "Joy offset is set to %f", joy_offset_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr blade_speed_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr blades_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    const int N_BLADES = 2;
    double max_speed_ = 200.0 / 60.0;
    double blades_speed_{0.0};
    int enable_button_;
    int speed_axis_;
    bool require_enable_button_{false};
    int start_button_;
    bool is_start_{false};
    double joy_offset_{0.0};
    double joy_deadzone_{0.15};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CFRBladesControl>());
    rclcpp::shutdown();
    return 0;
}