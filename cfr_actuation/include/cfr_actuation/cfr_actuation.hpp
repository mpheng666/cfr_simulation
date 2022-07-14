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
    struct MotorDegLimit
    {
        double LXmin {-16.5};
        double LXmax {16.5};
        double RXmin {-16.5};
        double RXmax {16.5};
        double RYmin {-12.5};
        double RYmax {16.5};
    };

    struct JoyLimit
    {
        double joymin {-0.66};
        double joymax {0.66};
    };

    struct MotorActuation
    {
        double LXMotordeg_FB {0.0};
        double RXMotordeg_FB {0.0};
        double LXMotordeg_ROT {0.0};
        double RXMotordeg_ROT {0.0};
        double LXMotordeg {0.0};
        double RXMotordeg {0.0};
        double RYMotordeg {0.0};

        void print()
        {
            printf("LXMotordeg_FB: %f \n", LXMotordeg_FB);
            printf("RXMotordeg_FB: %f \n", RXMotordeg_FB);
            printf("LXMotordeg_ROT: %f \n", LXMotordeg_ROT);
            printf("RXMotordeg_ROT: %f \n", RXMotordeg_ROT);
            printf("LXMotordeg: %f \n", LXMotordeg);
            printf("RXMotordeg: %f \n", RXMotordeg);
            printf("RYMotordeg: %f \n", RYMotordeg);
        };
    };

    class CfrActuation : public rclcpp::Node
    {
        public:
            CfrActuation();
            ~CfrActuation();

        private:
            MotorDegLimit motor_deg_limit_;
            JoyLimit joy_limit_;
            MotorActuation motor_actuation_;

            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_pub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
            rclcpp::TimerBase::SharedPtr timer_;

            void loadParams();
            void timerCb();
            void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
            void joyToMotorActuation(const double JoystickLeftX, const double JoystickLeftY, const double JoystickRightX, const double JoystickRightY);
    };

} // cfr_actuation_ns

#endif