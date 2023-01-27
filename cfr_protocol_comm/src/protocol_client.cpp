#include "cfr_protocol_comm/protocol_client.hpp"

namespace cfr_protocol {
    ProtocolClient::ProtocolClient()
        : Node("cfr_protocol_client")
        , protocol_client_(
          this->create_client<cfr_protocol_interfaces::srv::TriggerService>(
          "/cfr_protocol_server/cfr_protocol_service"))
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

    bool ProtocolClient::sendCommand(const std::string& input)
    {
        std::vector<std::string> tokens{};
        if (ProtocolMessageHandler::process(input, tokens)) {
            Request_SP req = processTokenIntoReq(tokens);
            sendProtocol(req);
            return true;
        }
        return false;
    }

    Request_SP ProtocolClient::processTokenIntoReq(const std::vector<std::string>& tokens)
    {
        const std::string service_name = tokens.at(0);
        std::vector<double> data{};
        // for (const auto& e : data) {
        //     std::cout << e << " ";
        // }
        // std::cout << "\n";
        if (tokens.size() > 1) {
            std::transform(tokens.cbegin() + 1, tokens.cend(), std::back_inserter(data),
                           [](const auto& v) {
                               double val = 0.0;
                               try {
                                   val = std::stod(v);
                               }
                               catch (const std::exception& e) {
                                   std::cerr << e.what() << '\n';
                               }
                               return std::stod(v);
                           });
        }
        auto request =
        std::make_shared<cfr_protocol_interfaces::srv::TriggerService::Request>();
        request->service_name = service_name;
        request->data = data;

        return request;
    }

    void ProtocolClient::sendProtocol(const Request_SP request)
    {
        auto result_future = protocol_client_->async_send_request(request);
        if (result_future.valid()) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Service " << request->service_name << " ok");
            auto result = result_future.get();
            RCLCPP_INFO_STREAM(this->get_logger(), "Result: " << result->success);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service \n");
        }
    }
} // namespace cfr_protocol