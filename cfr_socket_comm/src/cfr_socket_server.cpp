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
            std::string con_notif =
            "Connection from: " +
            con_handle->socket.remote_endpoint().address().to_string() + "\n\n";
            std::cout << con_notif;
            doAsynRead(con_handle);
        }
        else {
            std::cerr << "Accept error: " << err.message() << "\n";
            connections_.erase(con_handle);
        }
        startAccept();
    }

    void CFRSocketServer::doAsynWrite(con_handle_t con_handle,
                                      const std::shared_ptr<std::string> buff)
    {
        auto handler = boost::bind(&CFRSocketServer::handleWrite, this, con_handle, buff,
                                   boost::asio::placeholders::error);
        boost::asio::async_write(con_handle->socket, boost::asio::buffer(*buff), handler);
    }

    void
    CFRSocketServer::handleWrite(con_handle_t con_handle,
                                 [[maybe_unused]] std::shared_ptr<std::string> msg_buffer,
                                 boost::system::error_code const& err)
    {
        if (err) {
            std::cerr << "Write error: " << err.message() << std::endl;
            connections_.erase(con_handle);
        }
        else {
            if (con_handle->socket.is_open()) {
                // Write completed successfully and connection is open
                std::cout << "Write: " << *msg_buffer;
            }
        }
        doAsynRead(con_handle);
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
        if (err) {
            std::cerr << "Read error: " << err.message() << std::endl;
            connections_.erase(con_handle);
        }

        if (bytes_transfered > 0) {
            std::istream is(&con_handle->read_buffer);
            std::string line;
            std::getline(is, line);

            auto respond_msg = std::make_shared<std::string>(line + ",OK" + "\n");
            std::cout << "Read: " << *respond_msg;

            doAsynWrite(con_handle, respond_msg);
        }
    }

} // namespace cfr_socket_comm