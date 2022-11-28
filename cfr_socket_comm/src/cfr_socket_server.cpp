#include "cfr_socket_comm/cfr_socket_server.hpp"

namespace cfr_socket_comm {
    CFRSocketServer::CFRSocketServer(boost::asio::io_context& io_context,
                                     const uint32_t port)
        : 
        // Node("cfr_scoket_node")
         port_(port)
        , acceptor_(io_context, tcp::endpoint(tcp::v4(), port_))
    {
    }

    void CFRSocketServer::start()
    {
        std::cout << "start \n";
        // RCLCPP_INFO(this->get_logger(), "Started server!");
        try {
            do_accept();
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void CFRSocketServer::do_accept()
    {
        acceptor_.async_accept([&](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<session>(std::move(socket))->start();
            }
            do_accept();
        });
    }
} // namespace cfr_socket_comm