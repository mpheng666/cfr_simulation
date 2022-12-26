#include "cfr_socket_comm/cfr_socket_client.hpp"

namespace cfr_socket_comm {
    CFRSocketClient::CFRSocketClient() {}

    void CFRSocketClient::start(const std::string_view& host /* = "localhost" */,
                                const std::string_view& port /* = "10000" */)
    {
        if (handleConnection(host, port)) {
            runClient();
        }
    }

    bool CFRSocketClient::handleConnection(const std::string_view& host,
                                           const std::string_view& port)
    {
        try {
            auto endpoints = tcp::resolver(io_context_).resolve(host, port);
            boost::asio::connect(socket_, endpoints);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    void CFRSocketClient::runClient()
    {
        for (;;) {
            handleWrite();
            handleRead();
        }
    }

    size_t CFRSocketClient::handleWrite()
    {
        std::cout << CLIENT_NAME_ << ": ";
        char request_data[MAX_BUFFER_SIZE_];
        std::cin.getline(request_data, MAX_BUFFER_SIZE_);
        const size_t request_size = std::strlen(request_data);
        request_data[request_size] = '\n';
        try {
            boost::asio::write(socket_, boost::asio::buffer(request_data, request_size+1));
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
        return request_size;
    }

    size_t CFRSocketClient::handleRead()
    {
        size_t buffer_size{};
        boost::asio::streambuf stream_read_buffer;
        try {
            buffer_size = boost::asio::read_until(socket_, stream_read_buffer, "\n");
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
        }
        std::string stream_read_buffer_str(
        (std::istreambuf_iterator<char>(&stream_read_buffer)),
        std::istreambuf_iterator<char>());
        // std::cout << HOST_NAME_ << ": ";
        std::cout << stream_read_buffer_str << "\n";

        return buffer_size;
    }

} // namespace cfr_socket_comm