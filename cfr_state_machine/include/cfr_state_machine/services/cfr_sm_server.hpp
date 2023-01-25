#ifndef CFR_SM_SERVER_HPP_
#define CFR_SM_SERVER_HPP_

// #include "cfr_state_machine/cfr_sm_service.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "cfr_state_machine/cfr_state_machine.hpp"

#include <memory>
#include <string>

namespace cfr_sm {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    class CFRControlServer : public rclcpp::Node {
    public:
        CFRControlServer();
        void start();

    private:
        sm_CFR cfr_sm_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr feedback_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_sub_;

        void controlCallback(/* const */ std_msgs::msg::Float32MultiArray::SharedPtr msg);
    };
} // namespace cfr_sm

#endif