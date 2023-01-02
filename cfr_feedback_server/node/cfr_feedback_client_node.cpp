#include <boost/asio.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>

using boost::asio::ip::tcp;

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: client_node <host> <port>\n";
        return 1;
    }

    boost::asio::io_context io_context;

    tcp::socket s(io_context);
    tcp::resolver resolver(io_context);
    boost::asio::connect(s, resolver.resolve(argv[1], argv[2]));

    for (;;) {
        try {

            std::string result;
            boost::asio::streambuf streambuf;
            boost::asio::read_until(s, streambuf, "\n");

            std::string stream_read_buffer_str(
            (std::istreambuf_iterator<char>(&streambuf)),
            std::istreambuf_iterator<char>());
            std::cout << stream_read_buffer_str << "\n";
        }
        catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
            return 1;
        }
    }

    return 0;
}