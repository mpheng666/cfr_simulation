#include "cfr_protocol_comm/protocol_client.hpp"
#include <cstring>
#include <future>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto protocol_client_node = std::make_shared<cfr_protocol::ProtocolClient>();
    executor.add_node(protocol_client_node);
    auto future = std::async(std::launch::async, [&]() { executor.spin(); });

    std::string input;

    while (rclcpp::ok()) {
        std::cout << "ED: ";
        std::getline(std::cin, input);
        if (input == "quit")
            break;
        // std::cout << "Read size: " << input.size() << "\n";
        protocol_client_node->sendCommand(input);
        input.clear();
    }

    rclcpp::shutdown();
    // std::cout << "Return \n";

    return 0;
}