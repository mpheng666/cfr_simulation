#include "cfr_socket_comm/cfr_socket_server.hpp"

namespace cfr_socket_comm {
    CFRSocketServer::CFRSocketServer(boost::asio::io_context& io_context,
                                     const uint32_t port)
        : port_(port)
        , acceptor_(io_context, tcp::endpoint(tcp::v4(), port_))
    {
    }

    void CFRSocketServer::start()
    {
        for (;;) {
            std::thread(&CFRSocketServer::runSession, this, acceptor_.accept()).detach();
        }
    }

    void CFRSocketServer::runSession(tcp::socket sock)
    {
        try {
            for (;;) {
                char data[MAX_BUFFER_SIZE_];

                boost::system::error_code error;
                size_t length = sock.read_some(boost::asio::buffer(data), error);
                if (error == boost::asio::error::eof)
                    break;
                else if (error)
                    throw boost::system::system_error(error);
                if (length) {
                    std::string data_str(data, length);
                    std::cout << data_str << "\n";
                }

                boost::asio::write(sock, boost::asio::buffer(data, length));
            }
        }
        catch (std::exception& e) {
            std::cerr << "Exception in thread: " << e.what() << "\n";
        }
    }
} // namespace cfr_socket_comm