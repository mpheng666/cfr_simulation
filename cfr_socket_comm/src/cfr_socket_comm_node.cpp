#include "cfr_socket_comm/cfr_socket_server.hpp"

int main(int argc, char** argv)
{
    // cfr_socket_comm::Server srv;
    // srv.listen(10000);
    // srv.run();
    boost::asio::io_context io_context;

    cfr_socket_comm::CFRSocketServer cfr_socket_server(io_context, 10000);

    cfr_socket_server.start(argc, argv);
    io_context.run();

    return 0;
}
