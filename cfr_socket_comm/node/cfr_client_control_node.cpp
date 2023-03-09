#include "cfr_socket_comm/cfr_client_control.hpp"
#include "rclcpp/rclcpp.hpp"

void handler(const boost::system::error_code& error, int signal_number)
{
    std::cout << "handling signal " << signal_number << std::endl;
    exit(1);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        if (argc != 4) {
            std::cerr << "Usage: client_node <host> <command_port> <feedback_port> \n";
            return 1;
        }

        boost::asio::io_context ioc;
        boost::asio::signal_set signals(ioc, SIGINT);

        auto node = std::make_shared<cfr_socket_comm::CfrClientControl>(argv[1], argv[2],
                                                                        argv[3], ioc);

        std::thread t([&ioc] { ioc.run(); });
        node->start();
        std::thread t2([&] { rclcpp::spin(node); });
        // rclcpp::spin(node);
        std::cout << "After signal";

        signals.async_wait(handler);
        ioc.run();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    rclcpp::shutdown();
    return 0;
}