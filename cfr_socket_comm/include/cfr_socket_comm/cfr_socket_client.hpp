#ifndef CFR_SOCKET_CLIENT_HPP_
#define CFR_SOCKET_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class CFRSocketClient {
    public:
        CFRSocketClient(const std::string& host, const std::string& service);
        void start();

    private:
        boost::asio::io_context io_context_;
        tcp::socket socket_{io_context_};
        static constexpr int MAX_BUFFER_SIZE_{1024};
        std::string host_{"localhost"};
        std::string port_{"10000"};

        void runClient();
    };
} // namespace cfr_socket_comm

#endif