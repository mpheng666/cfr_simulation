#include "cfr_socket_comm/cfr_socket_client.hpp"

namespace cfr_socket_comm {
    CFRSocketClient::CFRSocketClient(const std::string& host, const std::string& port)
        : host_(host)
        , port_(port)
    {
    }

    void CFRSocketClient::start()
    {
        try {
            auto endpoints = tcp::resolver(io_context_).resolve(host_, port_);
            boost::asio::connect(socket_, endpoints);
            runClient();
            // std::thread(&CFRSocketClient::runClient, this).detach();
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void CFRSocketClient::runClient()
    {
        try {
            for (;;) {
                std::cout << "External Device: ";
                char request[MAX_BUFFER_SIZE_];
                std::cin.getline(request, MAX_BUFFER_SIZE_);
                // if (request[0] == 'q') {
                //     return 0;
                // }
                size_t request_length = std::strlen(request);
                boost::asio::write(socket_, boost::asio::buffer(request, request_length));

                boost::system::error_code error;
                boost::asio::streambuf buffer;
                char reply[MAX_BUFFER_SIZE_];
                size_t reply_length =
                boost::asio::read(socket_, boost::asio::buffer(reply, request_length));
                std::cout << "CFR: ";
                std::string reply_str(reply, reply_length);
                std::cout << reply_str;
                std::cout << "\n";
            }
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

} // namespace cfr_socket_comm