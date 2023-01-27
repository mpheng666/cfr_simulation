#include "cfr_protocol_comm/protocol_client.hpp"

namespace cfr_protocol {
    ProtocolClient::ProtocolClient()
        : Node("cfr_protocol_client")
        , protocol_client_(
          this->create_client<cfr_protocol_interfaces::srv::TriggerService>(
          "~/protocol_client"))
        , axis_control_pub_(
          this->create_publisher<std_msgs::msg::Float32MultiArray>("~/axis_control", 10))
        , cmdvel_control_pub_(this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "~/cmdvel_control", 10))
    {
        start();
    }

    void ProtocolClient::start()
    {
        while (!protocol_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                             "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "service not available, waiting again...");
        }
    }
} // namespace cfr_protocol