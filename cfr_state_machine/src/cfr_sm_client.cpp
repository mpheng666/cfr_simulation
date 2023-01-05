#include "cfr_state_machine/cfr_sm_client.hpp"

namespace cfr_sm_client {
    CFRSMClient::CFRSMClient(const std::string& client_name)
        : Node("test_client_control_node")
        , client_name_(client_name)
        , control_pub_(
          this->create_publisher<std_msgs::msg::Float32MultiArray>("control_channel", 10))
    {
    }

    void CFRSMClient::callCFRService(const std::string& command)
    {
        auto client_node = rclcpp::Node::make_shared(client_name_);

        auto client = client_node->create_client<std_srvs::srv::Trigger>(
        CLIENT_NS_ + serviceToName(strCmdToService(command)));

        if(!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(client_node->get_logger(),
                             "client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(client_node->get_logger(), "waiting for service to appear...");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(client_node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(client_node->get_logger(), "service call failed :(");
            return;
        }
        auto result = result_future.get();
    }

    void CFRSMClient::callControl(const std::vector<float>& command)
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = command;
        control_pub_->publish(msg);
    }
}