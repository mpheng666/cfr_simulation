#include "cfr_socket_comm/cfr_socket_server.hpp"

int main(int argc, char ** argv)
{
    // rclcpp::init(argc, argv);

    boost::asio::io_context io_context;

    // std::shared_ptr<cfr_socket_comm::CFRSocketServer> cfr_socket_server =
    // std::make_shared<cfr_socket_comm::CFRSocketServer>(io_context, 10001);

    cfr_socket_comm::CFRSocketServer cfr_socket_server(io_context, 10001);

    cfr_socket_server.start();
    io_context.run();

    // rclcpp::spin(cfr_socket_server);
    std::cout << "End of spin";

    // rclcpp::shutdown();
    // std::cout << "shutting down";

    return 0;
}
