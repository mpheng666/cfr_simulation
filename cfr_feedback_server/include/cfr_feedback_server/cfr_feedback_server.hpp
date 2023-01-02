#ifndef CFR_FEEDBACK_SERVER_HPP_
#define CFR_FEEDBACK_SERVER_HPP_

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <ctime>
#include <iostream>
#include <string>

using boost::asio::ip::tcp;

namespace cfr_feedback_server {

    class tcp_connection : public boost::enable_shared_from_this<tcp_connection> {
    public:
        typedef boost::shared_ptr<tcp_connection> pointer;

        static pointer create(boost::asio::io_context& io_context)
        {
            return pointer(new tcp_connection(io_context));
        }

        tcp::socket& socket() { return socket_; }

        void start()
        {
            boost::asio::async_write(
            socket_, boost::asio::buffer("Hello from CFR feedback server \n"),
            boost::bind(&tcp_connection::handle_write, shared_from_this(),
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
        }

        void write(const std::string& msg)
        {
            const std::string delimiter{"\n"};

            boost::asio::async_write(
            socket_, boost::asio::buffer(msg + delimiter),
            boost::bind(&tcp_connection::handle_write, shared_from_this(),
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
        }

    private:
        tcp_connection(boost::asio::io_context& io_context)
            : socket_(io_context)
        {
        }

        void handle_write(const boost::system::error_code& /* error */,
                          size_t /* bytes_transferred */)
        {
            // std::cout << "Write bytes: " << bytes_transferred << "\n";
        }

        tcp::socket socket_;
        std::string message_;
    };
    class CFRFeedBackServer {
    public:
        CFRFeedBackServer(const int port);
        void run();
        bool write(const std::string& message);

    private:
        boost::asio::io_context io_context_;
        tcp::acceptor acceptor_;
        std::vector<tcp_connection::pointer> connections_;

        void start_accept();
        void handle_accept(tcp_connection::pointer new_connection,
                           const boost::system::error_code& error);
    };
} // namespace cfr_feedback_server

#endif