#include "cfr_socket_comm/cfr_socket_server.hpp"

namespace cfr_socket_comm {
    CFRSocketServer::CFRSocketServer(boost::asio::io_context& io_context,
                                     const uint32_t port)
        : port_(port)
        , acceptor_(io_context, tcp::endpoint(tcp::v4(), port_))
    {
    }

    void CFRSocketServer::start(int argc, char** argv)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("CFR_TCP_socket"),
                           "Started CFR TCP local host server at port " << port_);


        try {
            doAccept();
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    // void CFRSocketServer::callCFRServiceClient()
    // {
    //     rclcpp::init(argc, argv);
    //     cfr_sm_client::CFRSMClient client("cfr_sm_client");
    //     client.callCFRService();
    //     rclcpp::shutdown();
    // }

    void CFRSocketServer::doAccept()
    {
        acceptor_.async_accept([&](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<SocketSession>(std::move(socket))->doRead();
                // callCFRServiceClient(argc, argv, "init_service");
            }
            doAccept();
        });
    }
} // namespace cfr_socket_comm