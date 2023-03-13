#include "cfr_socket_comm/cfr_feedback_client.hpp"
#include "rclcpp/rclcpp.hpp"

using boost::asio::ip::tcp;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const std::string host = argv[1];
    const std::string port = argv[2];

    if (argc != 3) {
        std::cerr << "Usage: client_node <host> <port>\n";
        return 1;
    }

    boost::asio::io_context ioc;

    auto feedback_client_node =
    std::make_shared<cfr_socket_comm::CfrFeedbackClient>(ioc, host, port);

    std::thread spin_t([&]() { rclcpp::spin(feedback_client_node); });

    feedback_client_node->start();

    rclcpp::shutdown();

    return 0;
}