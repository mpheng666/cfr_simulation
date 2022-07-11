#ifndef _CFR_STATES_CFR_STATES_HPP_
#define _CFR_STATES_CFR_STATES_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_states_ns
{
   struct KinematicsStates
   {
        double acc_linear_x_curr {0.0};
        double acc_linear_y_curr {0.0};
        double acc_angular_z_curr {0.0};
        double velocity_linear_x_curr {0.0};
        double velocity_linear_y_curr {0.0};
        double velocity_angular_z_curr {0.0};
        // double position_linear_x_curr {0.0};
        // double position_linear_y_curr {0.0};
        // double position_angular_z_curr {0.0};

        // double acc_linear_x_prev {0.0};
        // double acc_linear_y_prev {0.0};
        // double acc_angular_z_prev {0.0};
        double velocity_linear_x_prev {0.0};
        double velocity_linear_y_prev {0.0};
        double velocity_angular_z_prev {0.0};
        // double position_linear_x_prev {0.0};
        // double position_linear_y_prev {0.0};
        // double position_angular_z_prev {0.0};
   };

    class CfrStates : public rclcpp::Node
    {
        public:
            CfrStates();
            ~CfrStates();

        private:
            KinematicsStates kinematics_states_;
            double t_prev_ {0.0};
            double t_ {0.0};

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr acc_sub_;
            rclcpp::TimerBase::SharedPtr timer_;

            void loadParams();
            void timerCb();
            void accelCb(const geometry_msgs::msg::Accel::SharedPtr msg);
            void updateStates();
    };

} // cfr_states_ns

#endif