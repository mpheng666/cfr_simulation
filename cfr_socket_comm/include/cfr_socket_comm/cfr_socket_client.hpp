#ifndef CFR_SOCKET_CLIENT_HPP_
#define CFR_SOCKET_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class CFRSocketClient {
    public:
        CFRSocketClient();
        void start(const std::string_view& host = "localhost", const std::string_view& port = "10000");

    private:
        boost::asio::io_context io_context_;
        tcp::socket socket_{io_context_};
        static constexpr int MAX_BUFFER_SIZE_{1024};
        static constexpr std::string_view CLIENT_NAME_{"ED"};
        static constexpr std::string_view HOST_NAME_{"CFR"};

        void runClient();
        bool handleConnection(const std::string_view& host, const std::string_view& port);
        size_t handleWrite();
        size_t handleRead();
    };
} // namespace cfr_socket_comm

#endif