#ifndef CFR_SOCKET_CLIENT_HPP_
#define CFR_SOCKET_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class CFRSocketClient {
        public:
            CFRSocketClient(boost::asio::io_context& io_context,
                            const tcp::resolver::results_type& endpoints);

            void write();
            void close();

        private:
            boost::asio::io_context& io_context_;
            tcp::socket socket_;

            void doConnect(const tcp::resolver::results_type& endpoints);
    };
} // namespace cfr_socket_comm

#endif