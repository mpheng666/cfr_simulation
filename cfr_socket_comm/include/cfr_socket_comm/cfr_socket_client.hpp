#ifndef CFR_SOCKET_CLIENT_HPP_
#define CFR_SOCKET_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class CFRSocketClient {
    public:
        CFRSocketClient();
        bool doConnect(const std::string_view& host, const std::string_view& port);
        size_t doWrite(const std::string& msg);
        std::string doRead();

    private:
        boost::asio::io_context ioc_;
        tcp::socket socket_{ioc_};
        static constexpr int MAX_BUFFER_SIZE_{1024};
    };
} // namespace cfr_socket_comm

#endif