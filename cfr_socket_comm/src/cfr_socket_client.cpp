#include "cfr_socket_comm/cfr_socket_client.hpp"

namespace cfr_socket_comm {
    CFRSocketClient::CFRSocketClient() {}

    bool CFRSocketClient::doConnect(const std::string_view& host, const std::string_view& port)
    {
        try {
            auto endpoints = tcp::resolver(ioc_).resolve(host, port);
            boost::asio::connect(socket_, endpoints);
        }
        catch (const std::exception& e) {
            std::cerr << "Resolve error: " << e.what() << '\n';
            return false;
        }
        return true;
    }

    size_t CFRSocketClient::doWrite(const std::string& msg)
    {
        size_t request_size{};
        try {
            request_size = boost::asio::write(socket_,
                               boost::asio::buffer(msg));
        }
        catch (const std::exception& e) {
            std::cerr << "Write error: " << e.what() << '\n';
        }
        return request_size;
    }

    std::string CFRSocketClient::doRead()
    {
        size_t buffer_size{};
        boost::asio::streambuf stream_read_buffer;
        try {
            buffer_size = boost::asio::read_until(socket_, stream_read_buffer, "\n");
        }
        catch (const std::exception& e) {
            std::cerr << "Read error: " << e.what() << "\n";
        }
        std::string stream_read_buffer_str(
        (std::istreambuf_iterator<char>(&stream_read_buffer)),
        std::istreambuf_iterator<char>());
        std::cout << stream_read_buffer_str << "\n";

        return stream_read_buffer_str;
    }

} // namespace cfr_socket_comm