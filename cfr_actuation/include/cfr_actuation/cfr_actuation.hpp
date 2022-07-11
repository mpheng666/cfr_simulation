#ifndef _CFR_ACTUATION_CFR_ACTUATION_HPP_
#define _CFR_ACTUATION_CFR_ACTUATION_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_actuation_ns
{

    struct MotorActuation
    {
        double left_x_motor_deg_fb;
        double right_x_motor_deg_fb;
        double left_x_motor_deg_rot;
        double right_x_motor_deg_rot;
        double left_x_motor_deg;
        double right_x_motor_deg;
        double right_y_motor_deg;

        void print()
        {
            printf("LXMotordeg_FB: %f \n", left_x_motor_deg_fb);
            printf("RXMotordeg_FB: %f \n", right_x_motor_deg_fb);
            printf("LXMotordeg_ROT: %f \n", left_x_motor_deg_rot);
            printf("RXMotordeg_ROT: %f \n", right_x_motor_deg_rot);
            printf("LXMotordeg: %f \n", left_x_motor_deg);
            printf("RXMotordeg: %f \n", right_x_motor_deg);
            printf("RYMotordeg: %f \n", right_y_motor_deg);
        };
    };

    class CfrActuation : public rclcpp::Node
    {
        public:
            CfrActuation();
            ~CfrActuation();

        private:
            double left_x_min_;
            double left_x_max_;
            double right_x_min_;
            double right_x_max_;
            double right_y_min_;
            double right_y_max_;

            MotorActuation motor_actuation_;

            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_pub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
            rclcpp::TimerBase::SharedPtr timer_;

            void loadParams();
            void timerCb();
            void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
            void joyToMotorActuation(const double joy_left_x, const double joy_left_y, const double joy_right_x, const double joy_right_y);
    };

}; // cfr_actuation_ns

#endif