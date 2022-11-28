#ifndef CFR_SM_CLIENT_HPP_
#define CFR_SM_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cfr_sm_client {
    enum class CFRSMServiceType { INIT, START, STOP, RESET };

    class CFRSMClient {
    public:
        CFRSMClient(const std::string& client_name):
        client_name_(client_name) {}

        void callCFRService(const CFRSMServiceType& service_type)
        {
            auto client_node = rclcpp::Node::make_shared(client_name_);
            auto client = client_node->create_client<std_srvs::srv::Trigger>(
            CLIENT_NS_ + serviceToName(service_type));

            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                    client_node->get_logger(),
                    "client interrupted while waiting for service to appear.");
                    return;
                }
                RCLCPP_INFO(client_node->get_logger(),
                            "waiting for service to appear...");
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

    private:
        const std::string CLIENT_NS_{"cfr_sm_node/"};
        std::string client_name_{};
        constexpr const char* serviceToName(CFRSMServiceType service_type) noexcept
        {
            switch (service_type) {
                case CFRSMServiceType::INIT:
                    return "init_service";
                case CFRSMServiceType::STOP:
                    return "stop_service";
                case CFRSMServiceType::START:
                    return "start_service";
                case CFRSMServiceType::RESET:
                    return "reset_service";
            }
        }
    };
} // namespace cfr_sm_client

#endif