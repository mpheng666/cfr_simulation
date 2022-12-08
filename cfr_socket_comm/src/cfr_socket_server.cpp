#include "cfr_socket_comm/cfr_socket_server.hpp"

namespace cfr_socket_comm {

    CFRSocketServer::CFRSocketServer()
        : io_context_()
        , acceptor_(io_context_)
        , connections_()
    {
    }

    void CFRSocketServer::listen(const uint32_t port /* = 10000 */)
    {
        auto endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port);
        acceptor_.open(endpoint.protocol());
        acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
        acceptor_.bind(endpoint);
        acceptor_.listen();
        startAccept();
    }

    void CFRSocketServer::run() { io_context_.run(); }

    void CFRSocketServer::startAccept()
    {
        auto con_handle = connections_.emplace(connections_.begin(), io_context_);
        auto handler = boost::bind(&CFRSocketServer::handleAccept, this, con_handle,
                                   boost::asio::placeholders::error);
        acceptor_.async_accept(con_handle->socket, handler);
    }

    void CFRSocketServer::handleAccept(con_handle_t con_handle,
                                       boost::system::error_code const& err)
    {
        if (!err) {
            std::cout << "Connection from: "
                      << con_handle->socket.remote_endpoint().address().to_string() << "\n";
            std::cout << "Sending message\n";
            auto buff = std::make_shared<std::string>("Hello World!\r\n\r\n");
            auto handler = boost::bind(&CFRSocketServer::handleWrite, this, con_handle,
                                       buff, boost::asio::placeholders::error);
            boost::asio::async_write(con_handle->socket, boost::asio::buffer(*buff),
                                     handler);
            doAsynRead(con_handle);
        }
        else {
            std::cerr << "We had an error: " << err.message() << std::endl;
            connections_.erase(con_handle);
        }
        startAccept();
    }

    void CFRSocketServer::handleWrite(con_handle_t con_handle,
                                      std::shared_ptr<std::string> msg_buffer,
                                      boost::system::error_code const& err)
    {
        if (!err) {
            std::cout << "Finished sending message\n";
            if (con_handle->socket.is_open()) {
                // Write completed successfully and connection is open
            }
        }
        else {
            std::cerr << "We had an error: " << err.message() << std::endl;
            connections_.erase(con_handle);
        }
    }

    void CFRSocketServer::doAsynRead(con_handle_t con_handle)
    {
        auto handler = boost::bind(&CFRSocketServer::handleRead, this, con_handle,
                                   boost::asio::placeholders::error,
                                   boost::asio::placeholders::bytes_transferred);
        boost::asio::async_read_until(con_handle->socket, con_handle->read_buffer, "\n",
                                      handler);
    }

    void CFRSocketServer::handleRead(con_handle_t con_handle,
                                     boost::system::error_code const& err,
                                     size_t bytes_transfered)
    {
        if (bytes_transfered > 0) {
            std::istream is(&con_handle->read_buffer);
            std::string line;
            std::getline(is, line);
            std::cout << "Message Received: " << line << std::endl;
        }

        if (!err) {
            doAsynRead(con_handle);
        }
        else {
            std::cerr << "We had an error: " << err.message() << std::endl;
            connections_.erase(con_handle);
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
                    std::cout << "CFR read: " << data_str << "\n";
                }
                char respond_data[3];
                respond_data[0] = 'O';
                respond_data[1] = 'K';
                respond_data[2] = '\n';

                boost::asio::write(sock, boost::asio::buffer(respond_data, 3));
            }
        }
        catch (std::exception& e) {
            std::cerr << "Exception in thread: " << e.what() << "\n";
        }
    }
} // namespace cfr_socket_comm