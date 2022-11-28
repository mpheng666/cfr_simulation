#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class session : public std::enable_shared_from_this<session> {
    public:
        session(tcp::socket socket)
            : socket_(std::move(socket))
        {
        }

        void start() { do_read(); }

    private:
        void do_read()
        {
            auto self(shared_from_this());
            socket_.async_read_some(
            boost::asio::buffer(data_, max_length),
            [this, self](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    do_write(length);
                }
            });
        }

        void do_write(std::size_t length)
        {
            auto self(shared_from_this());
            boost::asio::async_write(
            socket_, boost::asio::buffer(data_, length),
            [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                if (!ec) {
                    do_read();
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
        void start();

    private:
        uint32_t port_{};
        tcp::acceptor acceptor_;

        void do_accept();
    };
} // namespace cfr_socket_comm

#endif