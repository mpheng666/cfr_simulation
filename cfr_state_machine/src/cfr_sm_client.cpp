#include "cfr_state_machine/cfr_sm_client.hpp"

namespace cfr_sm_client {
    CFRSMClient::CFRSMClient(const std::string& client_name)
        : client_name_(client_name)
    {
    }

    void CFRSMClient::callCFRService(const std::string& command)
    {
        rclcpp::init(0, nullptr);
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
            rclcpp::shutdown();
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
        rclcpp::shutdown();
    }
}