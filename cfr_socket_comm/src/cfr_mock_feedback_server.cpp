#include "cfr_socket_comm/cfr_mock_feedback_server.hpp"

namespace cfr_socket_comm {

    CfrMockFeedbackServer::CfrMockFeedbackServer(const int port)
        : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
    {
        std::cout << "Started feedback server at port " << port << " \n";
        start_accept();
    }

    void CfrMockFeedbackServer::run() { io_context_.run(); }

    bool CfrMockFeedbackServer::write(const std::string& message)
    {
        for(const auto& connection : connections_)
        {
            connection->write(message);
            std::cout << "CFR: " << message;
        }

        return true;
    }

    void CfrMockFeedbackServer::start_accept()
    {
        tcp_connection::pointer new_connection = tcp_connection::create(io_context_);

        acceptor_.async_accept(new_connection->socket(),
                               boost::bind(&CfrMockFeedbackServer::handle_accept, this,
                                           new_connection,
                                           boost::asio::placeholders::error));
    }

    void CfrMockFeedbackServer::handle_accept(tcp_connection::pointer new_connection,
                                        const boost::system::error_code& error)
    {
        if (!error) {
            new_connection->start();
            connections_.push_back(new_connection);
            // TODO: Remove the connection if client disconnects
        }

        start_accept();
    }

} // namespace cfr_feedback_server