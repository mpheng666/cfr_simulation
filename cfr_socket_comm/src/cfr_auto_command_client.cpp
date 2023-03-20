#include "cfr_socket_comm/cfr_auto_command_client.hpp"

namespace cfr_socket_comm {
    CfrAutoCommandClient::CfrAutoCommandClient(boost::asio::io_context& ioc)
        : Node("cfr_auto_command_client")
        , ioc_(ioc)
        , twist_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "/cfr/cfr_mpc/cmd_vel",
          10,
          std::bind(&CfrAutoCommandClient::twistCb, this, std::placeholders::_1)))
        , blade_speed_sub_(this->create_subscription<std_msgs::msg::Int32>(
          "/cfr/auto_client/blade_speed",
          10,
          std::bind(&CfrAutoCommandClient::bladeSpeedCb, this, std::placeholders::_1)))
    {
        loadParams();
    }

    void CfrAutoCommandClient::start()
    {
        if (initCommandConnection()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Connection " << host_ << " at "
                                                                 << command_port_
                                                                 << " establised!");
            startSession();
        }
        else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Connection failed!");
        }
    }

    bool CfrAutoCommandClient::initCommandConnection()
    {
        try {
            auto endpoints = tcp::resolver(ioc_).resolve(host_, command_port_);
            boost::asio::connect(command_socket_, endpoints);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    void CfrAutoCommandClient::loadParams()
    {
        this->declare_parameter("commands");
        this->declare_parameter("host_ip");
        this->declare_parameter("command_port");

        this->get_parameter("commands", commands_);
        this->get_parameter("host_ip", host_);
        this->get_parameter("command_port", command_port_);

        RCLCPP_INFO_STREAM(this->get_logger(), "Load commands: ");
        for (const auto& command : commands_) {
            std::cout << command << " ";
        }
        std::cout << "\n";
    }

    void CfrAutoCommandClient::startSession()
    {
        for (const auto& command : commands_) {
            int do_command_count = 3;
            if (command == "START") {
                start_cfr_twist_ = true;
            }
            for (int i = 0; i < do_command_count; ++i) {
                std::cout << "ED: " << command << "\n";
                doCommandWrite(command + "\n");
                boost::asio::steady_timer t(ioc_,
                                            boost::asio::chrono::milliseconds(2000));
                t.wait();
                auto reply = doCommandRead();
                if (verifyReplyCommand(command, reply)) {
                    std::cout << "Command-> " << command << ": verified OK\n";
                    i = do_command_count;
                }
                else {
                    std::cout << "Command-> " << command << ": verified WARN\n";
                }
            }
        }

        while (rclcpp::ok()) {
            std::string val{};
            std::cout << "ED: ";
            std::cin >> val;
            val += '\n';
            doCommandWrite(val);
            auto reply = doCommandRead();
            std::cout << "CFR: " << reply;
        }
    }

    bool CfrAutoCommandClient::verifyReplyCommand(const std::string& command,
                                                  const std::string& reply)
    {
        auto tokens = ProtocolHandler::tokenizeCommandReply(reply);
        // std::cout << "Command verified: " << command << "\n";
        // for (const auto& token : tokens) {
        //     std::cout << "token: " << token << "\n";
        // }
        if (tokens.size()) {
            if (command == "PSTATE") {
                if (tokens.at(1) == "IDLE" || tokens.at(1) == "INITIALIZING" ||
                    tokens.at(1) == "READY" || tokens.at(1) == "RUNNING" ||
                    tokens.at(1) == "STOPPED" || tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "MODE") {
                if (tokens.at(1) == "MANUAL2" || tokens.at(1) == "NYP-AUTO" ||
                    tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "BEACONS") {
                if (tokens.size() == 14 || tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "NYPAUTO,1") {
                if (tokens.at(2) == "OK") {
                    return true;
                }
            }
            else if (command == "INIT") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "BLADEANG,10") {
                if (tokens.at(2) == "OK") {
                    return true;
                }
            }
            else if (command == "FB,1") {
                if (tokens.at(2) == "OK") {
                    return true;
                }
            }
            else if (command == "START") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "CTRL,0,0,0,0") {
                if (tokens.at(5) == "OK") {
                    return true;
                }
            }
            else if (command == "STOP") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
        }
        return false;
    }

    void CfrAutoCommandClient::doCommandWrite(const std::string& write_message)
    {
        command_socket_.async_write_some(
        boost::asio::buffer(write_message),
        [&]([[maybe_unused]] const boost::system::error_code& error,
            [[maybe_unused]] std::size_t bytes_transferred) {});
    }

    std::string CfrAutoCommandClient::doCommandRead()
    {
        std::string res{};
        boost::asio::streambuf stream_read_buffer;
        try {
            boost::asio::read_until(command_socket_, stream_read_buffer, "\n");
            res.append((std::istreambuf_iterator<char>(&stream_read_buffer)),
                       std::istreambuf_iterator<char>());
        }
        catch (const std::exception& e) {
            std::cerr << "Read error: " << e.what() << "\n";
        }
        std::cout << "CFR: " << res;
        return res;
    }

    void CfrAutoCommandClient::twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist_socket_msg_.linear_x_relative = msg->linear.x;
        twist_socket_msg_.linear_y_relative = msg->linear.y;
        twist_socket_msg_.angular_z_relative = msg->angular.z;
        twist_socket_msg_.blade_speed_rpm = 90;
        auto twist_socket_str =
        ProtocolHandler::makeStringTwistSocketFormat(twist_socket_msg_, DELIMITER_);
        // RCLCPP_INFO_STREAM(this->get_logger(), "ioc running: " << !ioc_.stopped());
        if (start_cfr_twist_) {
            doCommandWrite(twist_socket_str);
        }
    }

    void CfrAutoCommandClient::bladeSpeedCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        twist_socket_msg_.blade_speed_rpm = msg->data;
    }

} // namespace cfr_socket_comm
