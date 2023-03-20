#include "cfr_socket_comm/cfr_feedback_client.hpp"
#include "rclcpp/rclcpp.hpp"

using boost::asio::ip::tcp;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    boost::asio::io_context ioc;

    auto feedback_client_node =
    std::make_shared<cfr_socket_comm::CfrFeedbackClient>(ioc);

    std::thread spin_t([&]() { rclcpp::spin(feedback_client_node); });

    feedback_client_node->start();

    rclcpp::shutdown();

    return 0;
}