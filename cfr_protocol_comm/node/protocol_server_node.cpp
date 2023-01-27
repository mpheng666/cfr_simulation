#include "cfr_protocol_comm/protocol_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto protocol_server_node = std::make_shared<cfr_protocol::ProtocolServer>();

    rclcpp::spin(protocol_server_node);
    rclcpp::shutdown();

    return 0;
}