#include "cfr_socket_comm/cfr_socket_client.hpp"

namespace cfr_socket_comm {

    CFRSocketClient::CFRSocketClient(boost::asio::io_context& io_context,
                                     const tcp::resolver::results_type& endpoints)
        : io_context_(io_context)
        , socket_(io_context)
    {
        doConnect(endpoints);
    }

    void CFRSocketClient::doConnect(const tcp::resolver::results_type& endpoints)
    {
        boost::asio::async_connect(socket_, endpoints,
                                   [this](boost::system::error_code ec, tcp::endpoint) {
                                       if (!ec) {
                                           do_read_header();
                                       }
                                   });
    }
} // namespace cfr_socke_comm