#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "../lib/cfr_sm_client.hpp"
#include <boost/asio.hpp>

#include <chrono>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {
    class SocketSession : public std::enable_shared_from_this<SocketSession> {
    public:
        SocketSession(tcp::socket socket)
            : socket_(std::move(socket))
            , data_(MAX_DATA_LEN_)
        {
        }

        void doRead()
        {
            auto self(shared_from_this());
            socket_.async_read_some(
            boost::asio::buffer(data_, MAX_DATA_LEN_),
            [this, self](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    if (length) {
                        for (const auto& d : data_) {
                            std::cout << d;
                        }
                        std::cout << "\n";
                    }
                    // data_.clear();
                    // doRead();

                    // doWrite(length);
                }
            });
        }

        void doWrite(std::size_t length)
        {
            auto self(shared_from_this());

            boost::asio::async_write(
            socket_, boost::asio::buffer(data_, length),
            [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                if (!ec) {
                    doRead();
                }
            });
        }

    private:
        tcp::socket socket_;
        static constexpr size_t MAX_DATA_LEN_{1024};
        std::vector<char> data_;
    };

    class CFRSocketServer {
    public:
        CFRSocketServer(boost::asio::io_context& io_context, const uint32_t port = 10000);
        void start(int argc, char** argv);

    private:
        uint32_t port_{};
        tcp::acceptor acceptor_;

        // void callCFRServiceClient(int argc, char** argv, const std::string& service_name);
        void doAccept();
    };
} // namespace cfr_socket_comm

#endif