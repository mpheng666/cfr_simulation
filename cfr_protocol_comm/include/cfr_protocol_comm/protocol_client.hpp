#ifndef CFR_PROTOCOL_COMM_PROTOCOL_CLIENT_HPP_
#define CFR_PROTOCOL_COMM_PROTOCOL_CLIENT_HPP_

#include "cfr_protocol_comm/protocol.hpp"
#include "cfr_protocol_comm/protocol_validator.hpp"

#include "cfr_protocol_interfaces/srv/trigger_service.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace cfr_protocol {
    class ProtocolClient : public rclcpp::Node {
    public:
        ProtocolClient();

    private:
        rclcpp::Client<cfr_protocol_interfaces::srv::TriggerService>::SharedPtr protocol_client_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr axis_control_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        cmdvel_control_pub_;

        void start();

    };
} // namespace cfr_protocol

#endif