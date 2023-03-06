#ifndef _CFR_MPC_CFR_MPC_HPP_
#define _CFR_MPC_CFR_MPC_HPP_

#include "lib/mpcmoveCodeGeneration.h"
#include "lib/mpcmoveCodeGeneration_spec.h"
#include "lib/qpkwik.h"
#include "lib/rtwtypes.h"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_mpc {
    class CFRMPC : public rclcpp::Node {
    public:
        CFRMPC();
        ~CFRMPC();

    private:
        rclcpp::TimerBase::SharedPtr control_pub_timer_;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_control_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        sensor_msgs::msg::Joy joy_control_msg_;

        void argInit_3x1_real_T(double result[3]);
        void argInit_6x6_real_T(double result[36]);
        void argInit_72x1_boolean_T(boolean_T result[72]);
        boolean_T argInit_boolean_T();
        double argInit_real_T();
        void argInit_struct4_T(struct4_T* result);
        struct5_T argInit_struct5_T();
        struct6_T argInit_struct6_T();

        void MPCCompute(const geometry_msgs::msg::Twist& msg);
        void cmdvelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
        void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
        void controlPubCb();

        struct5_T r;
        double u[3];
    };

} // namespace cfr_mpc

#endif