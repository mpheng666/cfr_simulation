#include "rclcpp/rclcpp.hpp"
#include "cfr_socket_comm/cfr_client_control.hpp"

int main(int argc, char* argv[])
{
    try {
        if (argc != 3) {
            std::cerr << "Usage: client_node <host> <port>\n";
            return 1;
        }
        cfr_socket_comm::CfrClientControl cfr_client_control(argv[1], argv[2]);
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}