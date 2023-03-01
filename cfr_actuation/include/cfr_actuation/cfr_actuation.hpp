#ifndef _CFR_ACTUATION_CFR_ACTUATION_HPP_
#define _CFR_ACTUATION_CFR_ACTUATION_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_actuation_ns {
    struct JoyRemapper {
        int joy_left_x_axis{0};
        int joy_left_y_axis{1};
        int joy_right_x_axis{2};
        int joy_right_y_axis{3};
        double joy_left_x_magnitude{1.0};
        double joy_left_y_magnitude{1.0};
        double joy_right_x_magnitude{1.0};
        double joy_right_y_magnitude{1.0};
    };

    struct MotorDegLimit {
        double LXmin{-16.5};
        double LXmax{16.5};
        double RXmin{-16.5};
        double RXmax{16.5};
        double RYmin{-12.5};
        double RYmax{16.5};
    };

    struct JoyLimit {
        double joymin{-0.66};
        double joymax{0.66};
    };

    struct MotorActuation {
        double LXMotordeg_FB{0.0};
        double RXMotordeg_FB{0.0};
        double LXMotordeg_ROT{0.0};
        double RXMotordeg_ROT{0.0};
        double LXMotordeg{0.0};
        double RXMotordeg{0.0};
        double RYMotordeg{0.0};

        void print(const std::string& node_name) const
        {
            RCLCPP_INFO(rclcpp::get_logger(node_name), "LXMotordeg_FB: %f",
                        LXMotordeg_FB);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RXMotordeg_FB: %f",
                        RXMotordeg_FB);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "LXMotordeg_ROT: %f",
                        LXMotordeg_ROT);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RXMotordeg_ROT: %f",
                        RXMotordeg_ROT);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "LXMotordeg: %f", LXMotordeg);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RXMotordeg: %f", RXMotordeg);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RYMotordeg: %f", RYMotordeg);
        };
    };

    class CfrActuation : public rclcpp::Node {
    public:
        CfrActuation();

    private:
        JoyRemapper joy_remapper_;
        MotorDegLimit motor_deg_limit_;
        JoyLimit joy_limit_;
        MotorActuation motor_actuation_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_pub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allow_move_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr reset_timer_;

        bool allow_move_{false};
        bool reset_flag_{false};

        void loadParams();
        void timerCb();
        void resetTimerCb();
        void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
        void allowMoveCb(const std_msgs::msg::Bool::SharedPtr msg);
        void joyToMotorActuation(const double& JoystickLeftX,
                                 const double& JoystickLeftY,
                                 const double& JoystickRightX,
                                 const double& JoystickRightY);
    };

} // namespace cfr_actuation_ns

#endif