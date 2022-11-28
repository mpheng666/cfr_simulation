#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <boost/asio.hpp>

#include <chrono>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class SocketSession : public std::enable_shared_from_this<SocketSession> {
    public:
        SocketSession(tcp::socket socket)
            : socket_(std::move(socket))
        {
        }

        void start() { doRead(); }

    private:
        void doRead()
        {
            auto self(shared_from_this());
            socket_.async_read_some(
            boost::asio::buffer(data_, max_length),
            [this, self](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    doWrite(length);
                }
            });
        }

        void doWrite(std::size_t length)
        {
            auto self(shared_from_this());
            boost::asio::async_write(
            socket_, boost::asio::buffer(data_, length),
            [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                if (!ec) {
                    doRead();
                }
            });
        }

        tcp::socket socket_;
        enum { max_length = 1024 };
        char data_[max_length];
    };

    class CFRSocketServer {
    public:
        CFRSocketServer(boost::asio::io_context& io_context, const uint32_t port = 10000);
        void start(int argc, char** argv);

    private:
        uint32_t port_{};
        tcp::acceptor acceptor_;

        void setupCFRServiceClient(int argc, char** argv);
        void doAccept();
    };
} // namespace cfr_socket_comm

#endif