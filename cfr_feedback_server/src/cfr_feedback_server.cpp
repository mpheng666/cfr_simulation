#include "cfr_feedback_server/cfr_feedback_server.hpp"

namespace cfr_feedback_server {

    CFRFeedBackServer::CFRFeedBackServer(const int port)
        : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
    {
        std::cout << "Started feedback server at port " << port << " \n";
        start_accept();
    }

    void CFRFeedBackServer::run() { io_context_.run(); }

    bool CFRFeedBackServer::write(const std::string& message)
    {
        for(const auto& connection : connections_)
        {
            connection->write(message);
        }

        return true;
    }

    void CFRFeedBackServer::start_accept()
    {
        tcp_connection::pointer new_connection = tcp_connection::create(io_context_);

        acceptor_.async_accept(new_connection->socket(),
                               boost::bind(&CFRFeedBackServer::handle_accept, this,
                                           new_connection,
                                           boost::asio::placeholders::error));
    }

    void CFRFeedBackServer::handle_accept(tcp_connection::pointer new_connection,
                                        const boost::system::error_code& error)
    {
        if (!error) {
            new_connection->start();
            connections_.push_back(new_connection);
            std::cout << "Push back new connection! \n";
        }

        start_accept();
    }

} // namespace cfr_feedback_server