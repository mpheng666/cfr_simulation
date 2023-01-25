#ifndef CFR_SM_CFR_MANAGER_HPP_
#define CFR_SM_CFR_MANAGER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "cfr_manager/cfr_feedback_broadcastor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

namespace cfr_manager {
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    class CFRManager : public rclcpp::Node {
    public:
        CFRManager();
        void startBroadcastRobotStatus();
        void startEngine();
        void initialise();
        void setAllowMoveBlade(/* const */ bool command);
        void setAllowCmdvel(/* const */ bool command);
        void setBladeSpeed(/* const */ float command);
        void setCmdvel(/* const */ float x, /* const */ float y, /* const */ float z);
        void stopAllActions();
        void killAllActions();

    private:
        bool allow_move_blade_{false};
        bool allow_cmd_vel_{false};
        float blade_speed_{0.0f};
        int server_port_{10001};

        CFRFeedBackBroadcastor feedback_broadcastor_;
        static constexpr double BROADCAST_FREQUENCY_{10.0};
        CFRFeedbackMsg feedback_msg_{};

        nav_msgs::msg::Odometry odom_curr_;
        geometry_msgs::msg::Twist twist_curr_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr blade_speed_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        void
        odomCallback(/* const */ nav_msgs::msg::Odometry::SharedPtr msg);
        void pubTimerCallback();
        void updateTimerCallback();
    };
} // namespace cfr_manager

#endif