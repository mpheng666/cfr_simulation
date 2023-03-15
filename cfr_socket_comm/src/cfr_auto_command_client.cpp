#include "cfr_socket_comm/cfr_auto_command_client.hpp"

namespace cfr_socket_comm {
    CfrAutoCommandClient::CfrAutoCommandClient(const std::string& host,
                                               const std::string& command_port,
                                               boost::asio::io_context& ioc)
        : Node("cfr_auto_command_client")
        , host_(host)
        , command_port_(command_port)
        , ioc_(ioc)
        , twist_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "/cfr/cfr_mpc/cmd_vel",
          10,
          std::bind(&CfrAutoCommandClient::twistCb, this, std::placeholders::_1)))
    {
    }

    void CfrAutoCommandClient::start()
    {
        if (initCommandConnection()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Connection establised!");
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
        }
        return true;
    }

    void CfrAutoCommandClient::loadCommands()
    {
        this->get_parameter("commands", commands_);
    }

    void CfrAutoCommandClient::startSession()
    {
        for (const auto& command : commands_) {
            int do_command_count = 3;
            if(command == "START")
            {
                start_cfr_twist_ = true;
            }
            for (int i = 0; i < do_command_count; ++i) {
                std::cout << "ED: " << command << "\n";
                doCommandWrite(command + "\n");
                boost::asio::steady_timer t(ioc_,
                                            boost::asio::chrono::milliseconds(1000));
                t.wait();
                if (verifyReplyCommand(command, doCommandRead())) {
                    break;
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
        if (tokens.size() && tokens.at(0) == command) {
            if (command == "PSTATE") {
                if (tokens.at(1) == "IDLE" || tokens.at(1) == "INITIALIZING" ||
                    tokens.at(1) == "READY" || tokens.at(1) == "RUNNING" ||
                    tokens.at(1) == "STOPPED") {
                    return true;
                }
            }
            else if (command == "MODE") {
                if (tokens.at(1) == "MANUAL2" || tokens.at(1) == "NYP-AUTO") {
                    return true;
                }
            }
            else if (command == "BEACONS") {
                if (tokens.size() == 14) {
                    return true;
                }
            }
            else if (command == "NYPAUTO,1") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "INIT") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "BLADEANG,10") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "FB,1") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "START") {
                if (tokens.at(1) == "OK") {
                    return true;
                }
            }
            else if (command == "CTRL,0,0,0") {
                if (tokens.at(1) == "OK") {
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
        boost::asio::write(command_socket_, boost::asio::buffer(write_message));
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

    void CfrAutoCommandClient::bladeSpeedCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        twist_socket_msg_.blade_speed_rpm = msg->data;
    }

} // namespace cfr_socket_comm