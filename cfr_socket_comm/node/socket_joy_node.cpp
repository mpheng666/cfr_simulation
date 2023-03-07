// #include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <iostream>

namespace cfr_socket_comm {
    using boost::asio::ip::tcp;
    class SocketJoy {
    public:
        SocketJoy(const std::string& host, const std::string& port)
        {

            if (handleConnection(host, port)) {
                startSession();
            }
        }

    private:
        boost::asio::io_context ioc_;
        tcp::socket socket_{ioc_};
        static constexpr int MAX_BUFFER_SIZE{1024};

        bool handleConnection(const std::string& host, const std::string& port)
        {
            std::cout << "Handling connection \n";
            try {
                auto endpoints = tcp::resolver(ioc_).resolve(host, port);
                boost::asio::connect(socket_, endpoints);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
                return false;
            }
            return true;
        }

        void startSession()
        {
            std::cout << "Session started! \n";
            if (doHandShake()) {
                for (;;) {
                    doControlCommand(50, 0.1, 0.1, 0.1);
                    boost::asio::steady_timer t(ioc_,
                                                boost::asio::chrono::milliseconds(100));
                    t.wait();
                }
            }
            else {
                std::cerr << "Error during handshake, please try again later! \n";
                return;
            }
        }

        bool doHandShake()
        {
            std::cout << "Handshaking \n";
            return (doCommand("INIT", 3, 1000) && doCommand("START", 3, 1000) &&
                    doCommand("FB", 3, 1000) && doCommand("STARTENGINE", 3, 1000));
        }

        std::string doRead()
        {
            std::string res{};
            boost::asio::streambuf stream_read_buffer;
            try {
                boost::asio::read_until(socket_, stream_read_buffer, "\n");
                res.append((std::istreambuf_iterator<char>(&stream_read_buffer)),
                           std::istreambuf_iterator<char>());
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << "\n";
            }
            std::cout << "Read: " << res;
            return res;
        }

        void doWrite(const std::string& msg)
        {
            try {
                std::cout << "Write: " << msg;
                boost::asio::write(socket_, boost::asio::buffer(msg));
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

         static bool validateMsg(const std::string& msg, const std::string& target_msg)
        {
            std::cout << "Validating respond: " << target_msg;
            return msg + ",OK\n" == target_msg;
        }

        bool
        doCommand(const std::string& msg, const int retry_max_cout, int wait_duration_ms)
        {
            std::cout << "Doing command: " << msg << "\n";
            for (int i = 0; i <= retry_max_cout; ++i) {
                doWrite(msg + "\n");
                boost::asio::steady_timer t(
                ioc_, boost::asio::chrono::milliseconds(wait_duration_ms));
                t.wait();
                if (validateMsg(msg, doRead())) {
                    return true;
                }
            }
            return false;
        }

        bool doControlCommand(double blade_speed,
                              double linear_x,
                              double angular_z,
                              double linear_y)
        {
            std::string command = "CTRL";
            command.append(",")
            .append(std::to_string(blade_speed))
            .append(",")
            .append(std::to_string(linear_x))
            .append(",")
            .append(std::to_string(angular_z))
            .append(",")
            .append(std::to_string(linear_y))
            .append("\n");
            doWrite(command);
            return true;
        }
    };

} // namespace cfr_socket_comm

int main(int argc, char* argv[])
{
    try {
        if (argc != 3) {
            std::cerr << "Usage: client_node <host> <port>\n";
            return 1;
        }
        cfr_socket_comm::SocketJoy joyclient(argv[1], argv[2]);
    }
    catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}