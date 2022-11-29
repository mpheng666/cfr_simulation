#ifndef CFR_SOCKET_SERVER_HPP_
#define CFR_SOCKET_SERVER_HPP_

#include "../lib/cfr_sm_client.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <chrono>

using boost::asio::ip::tcp;

namespace cfr_socket_comm {

    struct Connection {
        boost::asio::ip::tcp::socket socket;
        boost::asio::streambuf read_buffer;
        Connection(boost::asio::io_service& io_service)
            : socket(io_service)
            , read_buffer()
        {
        }
        Connection(boost::asio::io_service& io_service, size_t max_buffer_size)
            : socket(io_service)
            , read_buffer(max_buffer_size)
        {
        }
    };

    class Server {
        boost::asio::io_service m_ioservice;
        boost::asio::ip::tcp::acceptor m_acceptor;
        std::list<Connection> m_connections;
        using con_handle_t = std::list<Connection>::iterator;

    public:
        Server()
            : m_ioservice()
            , m_acceptor(m_ioservice)
            , m_connections()
        {
        }

        void handle_read(con_handle_t con_handle,
                         boost::system::error_code const& err,
                         size_t bytes_transfered)
        {
            if (bytes_transfered > 0) {
                std::istream is(&con_handle->read_buffer);
                std::string line;
                std::getline(is, line);
                std::cout << "Message Received: " << line << std::endl;
            }

            if (!err) {
                do_async_read(con_handle);
            }
            else {
                std::cerr << "We had an error: " << err.message() << std::endl;
                m_connections.erase(con_handle);
            }
        }

        void do_async_read(con_handle_t con_handle)
        {
            auto handler = boost::bind(&Server::handle_read, this, con_handle,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred);
            boost::asio::async_read_until(con_handle->socket, con_handle->read_buffer,
                                          "\n", handler);
        }

        void handle_write(con_handle_t con_handle,
                          std::shared_ptr<std::string> msg_buffer,
                          boost::system::error_code const& err)
        {
            if (!err) {
                std::cout << "Finished sending message\n";
                if (con_handle->socket.is_open()) {
                    // Write completed successfully and connection is open
                }
            }
            else {
                std::cerr << "We had an error: " << err.message() << std::endl;
                m_connections.erase(con_handle);
            }
        }

        void handle_accept(con_handle_t con_handle, boost::system::error_code const& err)
        {
            if (!err) {
                std::cout << "Connection from: "
                          << con_handle->socket.remote_endpoint().address().to_string()
                          << "\n";
                std::cout << "Sending message\n";
                auto buff = std::make_shared<std::string>("Hello World!\r\n\r\n");
                auto handler = boost::bind(&Server::handle_write, this, con_handle, buff,
                                           boost::asio::placeholders::error);
                boost::asio::async_write(con_handle->socket, boost::asio::buffer(*buff),
                                         handler);
                do_async_read(con_handle);
            }
            else {
                std::cerr << "We had an error: " << err.message() << std::endl;
                m_connections.erase(con_handle);
            }
            start_accept();
        }

        void start_accept()
        {
            auto con_handle = m_connections.emplace(m_connections.begin(), m_ioservice);
            auto handler = boost::bind(&Server::handle_accept, this, con_handle,
                                       boost::asio::placeholders::error);
            m_acceptor.async_accept(con_handle->socket, handler);
        }

        void listen(uint16_t port)
        {
            auto endpoint =
            boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port);
            m_acceptor.open(endpoint.protocol());
            m_acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
            m_acceptor.bind(endpoint);
            m_acceptor.listen();
            start_accept();
        }

        void run() { m_ioservice.run(); }
    };
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
            std::string read_buffer{};
            socket_.async_read_some(
            boost::asio::buffer(read_buffer),
            [this, self, &read_buffer](boost::system::error_code ec, std::size_t length) {
                if (!ec) {
                    if (read_buffer.size()) {
                        std::cout << "length read: " << read_buffer.size() << "\n";
                    }
                    // if (length) {
                    //     for (const auto& d : read_buffer) {
                    //         std::cout << d;
                    //     }
                    //     std::cout << "\n";
                    // }
                    doWrite(read_buffer);
                }
            });
        }

        void doWrite(const std::string& read_buffer)
        {
            auto self(shared_from_this());
            std::string respond{"OK \n"};

            boost::asio::async_write(
            socket_, boost::asio::buffer(respond),
            [this, self, &read_buffer](boost::system::error_code ec,
                                       std::size_t length_write) {
                if (!ec) {
                    // if(read_buffer.size())
                    // {

                    // }
                    doRead();
                }
            });
        }

    private:
        tcp::socket socket_;
        static constexpr size_t MAX_DATA_LEN_{1024};
        std::vector<char> data_;
        std::string request{};
    };

    class CFRSocketServer {
    public:
        CFRSocketServer(boost::asio::io_context& io_context, const uint32_t port = 10000);
        void start(int argc, char** argv);

    private:
        uint32_t port_{};
        tcp::acceptor acceptor_;

        // void callCFRServiceClient(int argc, char** argv, const std::string&
        // service_name);
        void doAccept();
    };
} // namespace cfr_socket_comm

#endif