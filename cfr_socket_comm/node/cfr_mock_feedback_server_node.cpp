#include "cfr_socket_comm/cfr_mock_feedback_server.hpp"
#include "cfr_socket_comm/cfr_protocol_handler.hpp"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <thread>
#include <random>

void my_handler(int s)
{
    printf("Caught signal %d\n", s);
    exit(1);
}

int main(int argc, char* argv[])
{
    using namespace std::chrono_literals;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    static constexpr char DELIMITER{'\n'};
    try {
        if (argc != 2) {
            std::cerr << "Usage: server_node <port>\n";
            return 1;
        }

        cfr_socket_comm::CfrMockFeedbackServer mock_feedback_server(std::atoi(argv[1]));
        std::thread t(&cfr_socket_comm::CfrMockFeedbackServer::run, &mock_feedback_server
        );

        cfr_socket_comm::CFRFeedbackSocketFormat msg;

        for (;;) {
            std::random_device r;
            std::default_random_engine e1(r());
            std::uniform_int_distribution<int> uniform_dist(0, 6);
            msg.timestamped_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
            msg.position_x_m = static_cast<double>(uniform_dist(e1));
            msg.position_y_m = static_cast<double>(uniform_dist(e1));
            msg.theta_deg = static_cast<double>(uniform_dist(e1));
            auto msg_str =
            cfr_socket_comm::ProtocolHandler::makeStringCFRFeedbackSocketFormat(
            msg, DELIMITER);
            mock_feedback_server.write(msg_str);
            std::this_thread::sleep_for(80ms);
        }
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}