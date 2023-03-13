#include "cfr_socket_comm/cfr_auto_command_client.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        if (argc != 3) {
            std::cerr << "Usage: client_node <host> <command_port> \n";
            return 1;
        }

        boost::asio::io_context ioc;

        auto node =
        std::make_shared<cfr_socket_comm::CfrAutoCommandClient>(argv[1], argv[2], ioc);

        std::thread t([&ioc] { ioc.run(); });
        std::thread t2([&] { rclcpp::spin(node); });
        node->start();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    rclcpp::shutdown();
    return 0;
}