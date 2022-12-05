#include "cfr_socket_comm/cfr_socket_client.hpp"

int main(int argc, char* argv[])
{
    try {
        if (argc != 3) {
            std::cerr << "Usage: client_node <host> <port>\n";
            return 1;
        }

        cfr_socket_comm::CFRSocketClient client(argv[1], argv[2]);
        client.start();
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}