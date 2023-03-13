#include "cfr_socket_comm/cfr_socket_server.hpp"

int main(int argc, char** argv)
{
    static constexpr int DEFAULT_PORT {10000};
    cfr_socket_comm::CFRSocketServer cfr_socket_server{};
    if (argc > 1) {
        cfr_socket_server.listen(std::atoi(argv[1]));
        std::cout << "Running localhost server, listening to port " << std::atoi(argv[1]) << "\n";
    }
    else {
        cfr_socket_server.listen(DEFAULT_PORT);
        std::cout << "Running localhost server, listening to default port " << DEFAULT_PORT << "\n";
    }
    cfr_socket_server.run();

    return 0;
}
