#ifndef CFR_PROTOCOL_COMM_PROTOCOL_SERVER_HPP_
#define CFR_PROTOCOL_COMM_PROTOCOL_SERVER_HPP_

#include "cfr_protocol_comm/protocol.hpp"
#include "cfr_protocol_interfaces/srv/trigger_service.hpp"

#include "cfr_state_machine/cfr_state_machine.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace cfr_protocol {
    using std::placeholders::_1;
    using std::placeholders::_2;

    class ProtocolServer : public rclcpp::Node {
    public:
        ProtocolServer();

    private:
        cfr_sm::sm_CFR robot_state_machine_;
        rclcpp::Service<cfr_protocol_interfaces::srv::TriggerService>::SharedPtr
        protocol_service_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        axis_control_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
        cmdvel_control_sub_;

        void protocolServiceCb(
        const std::shared_ptr<cfr_protocol_interfaces::srv::TriggerService::Request> req,
        std::shared_ptr<cfr_protocol_interfaces::srv::TriggerService::Response> res);

        void AxisControlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void CmdVelControlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        void getSMState();

        void triggerEvent(const std::string& service_name, const std::vector<double>& data);
    };
} // namespace cfr_protocol

#endif