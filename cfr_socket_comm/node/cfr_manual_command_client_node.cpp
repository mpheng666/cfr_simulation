#include "cfr_socket_comm/cfr_socket_client.hpp"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void my_handler(int s)
{
    printf("Caught signal %d\n", s);
    exit(1);
}

int main(int argc, char* argv[])
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    static constexpr char DELIMITER{'\n'};
    try {
        if (argc != 3) {
            std::cerr << "Usage: client_node <host> <port>\n";
            return 1;
        }

        cfr_socket_comm::CFRSocketClient client;
        client.doConnect(argv[1], argv[2]);
        for (;;) {
            std::string val{};
            std::cout << "ED: ";
            std::cin >> val;
            val += DELIMITER;
            client.doWrite(val);
            auto reply = client.doRead();
            std::cout << "CFR: " << reply;
        }
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}