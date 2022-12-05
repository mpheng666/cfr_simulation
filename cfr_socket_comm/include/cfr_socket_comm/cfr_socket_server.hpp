#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "../lib/cfr_sm_client.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {

    class CFRSocketServer {
    public:
        CFRSocketServer(boost::asio::io_context& io_context, const uint32_t port = 10000);
        void start();

    private:
        uint32_t port_{};
        tcp::acceptor acceptor_;
        static constexpr int MAX_BUFFER_SIZE_{1024};

        void runSession(tcp::socket sock);
    };

} // namespace cfr_socket_comm

#endif