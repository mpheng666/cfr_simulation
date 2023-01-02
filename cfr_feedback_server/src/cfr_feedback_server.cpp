#include "cfr_feedback_server/cfr_feedback_server.hpp"

namespace cfr_feedback_server {
    CFRFeedBackServer::CFRFeedBackServer(const int port)
        : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port))
        , socket_(io_context_)
    {
        try {
            acceptor_.accept(socket_);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    bool CFRFeedBackServer::write(const std::string& message)
    {
        size_t written_size{};
        try {
            boost::system::error_code ignored_error;
            written_size =
            boost::asio::write(socket_, boost::asio::buffer(message), ignored_error);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }

        return message.size() == written_size;
    }

} // namespace cfr_feedback_server