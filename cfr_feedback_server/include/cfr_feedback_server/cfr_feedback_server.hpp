#ifndef CFR_FEEDBACK_SERVER_HPP_
#define CFR_FEEDBACK_SERVER_HPP_

#include <boost/asio.hpp>
#include <iostream>
#include <string>

using boost::asio::ip::tcp;

namespace cfr_feedback_server {
    class CFRFeedBackServer {
    public:
        CFRFeedBackServer(const int port);
        bool write(const std::string& message);

    private:
        boost::asio::io_context io_context_;
        tcp::acceptor acceptor_;
        tcp::socket socket_;
    };
} // namespace cfr_feedback_server

#endif