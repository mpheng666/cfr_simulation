#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "../lib/cfr_sm_client.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {

    struct Connection {
        boost::asio::ip::tcp::socket socket;
        boost::asio::streambuf read_buffer;
        Connection(boost::asio::io_context& io_context)
            : socket(io_context)
            , read_buffer()
        {
        }
        Connection(boost::asio::io_context& io_context, size_t max_buffer_size)
            : socket(io_context)
            , read_buffer(max_buffer_size)
        {
        }
    };

    class CFRSocketServer {
        using con_handle_t = std::list<Connection>::iterator;

    public:
        CFRSocketServer();
        void listen(const uint32_t port = 10000);
        void run();

    private:
        boost::asio::io_context io_context_;
        tcp::acceptor acceptor_;
        std::list<Connection> connections_;
        static constexpr int MAX_BUFFER_SIZE_{1024};

        void startAccept();
        void handleAccept(con_handle_t con_handle, boost::system::error_code const& err);
        void doAsynWrite(con_handle_t con_handle,
                         const std::shared_ptr<std::string> buff);
        void handleWrite(con_handle_t con_handle,
                         std::shared_ptr<std::string> msg_buffer,
                         boost::system::error_code const& err);
        void doAsynRead(con_handle_t con_handle);
        void handleRead(con_handle_t con_handle,
                        boost::system::error_code const& err,
                        size_t bytes_transfered);

        void runSession(tcp::socket sock);
    };

} // namespace cfr_socket_comm

#endif