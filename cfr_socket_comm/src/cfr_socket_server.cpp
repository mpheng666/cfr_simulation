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

        callCFRServiceClient(argc, argv, "init_service");

        try {
            doAccept();
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void CFRSocketServer::callCFRServiceClient(int argc, char** argv, const std::string& service_name)
    {
        rclcpp::init(argc, argv);
        auto client_node = rclcpp::Node::make_shared("CFR_sm_service_client");
        auto client =
        client_node->create_client<std_srvs::srv::Trigger>("cfr_sm_node/" + service_name);

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(client_node->get_logger(),
                             "client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(client_node->get_logger(), "waiting for service to appear...");
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(client_node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(client_node->get_logger(), "service call failed :(");
            return ;
        }

        auto result = result_future.get();
        rclcpp::shutdown();
    }

    void CFRSocketServer::doAccept()
    {
        acceptor_.async_accept([&](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<SocketSession>(std::move(socket))->doRead();
            }
            doAccept();
        });
    }
} // namespace cfr_socket_comm