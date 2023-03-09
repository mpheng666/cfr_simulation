#include "cfr_socket_comm/cfr_client_control.hpp"

namespace cfr_socket_comm {
    CfrClientControl::CfrClientControl(const std::string& host,
                                       const std::string& command_port,
                                       const std::string& feedback_port,
                                       boost::asio::io_context& ioc)
        : Node("cfr_client_control")
        , host_(host)
        , command_port_(command_port)
        , feedback_port_(feedback_port)
        , ioc_(ioc)
        , odom_pub_(this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10))
        , twist_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "~/cmd_vel",
          10,
          std::bind(&CfrClientControl::twistCb, this, std::placeholders::_1)))
        , server_command_timer_(this->create_wall_timer(
          std::chrono::duration(std::chrono::milliseconds(100)),
          std::bind(&CfrClientControl::serverTwistCommandCb, this)))
    {
    }

    void CfrClientControl::start()
    {
        ioc_.run();
        if (initCommandConnection()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Connection establised!");
            startSession();
        }
        else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Connection failed!");
        }
    }

    bool CfrClientControl::initCommandConnection()
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

    bool CfrClientControl::initFeedbackConnection()
    {
        try {
            auto endpoints = tcp::resolver(ioc_).resolve(host_, feedback_port_);
            boost::asio::connect(feedback_socket_, endpoints);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
        return true;
    }

    bool CfrClientControl::initCFREngine()
    {
        doCommandWrite("PSTATE\n");
        doCommandWrite("MODE\n");
        doCommandWrite("BEACONS\n");
        doCommandWrite("NYPAUTO,1\n");
        doCommandWrite("INIT\n");
        doCommandWrite("PSTATE\n");
        doCommandWrite("PSTATE\n");
        doCommandWrite("BLADEANG, 10\n");
        doCommandWrite("FB, 1\n");
        doCommandWrite("START\n");
        doCommandWrite("PSTATE\n");
        // send CTRL,0,0,0,0
        doCommandWrite("STARTENGINE\n");
        // continue to send
        doCommandWrite("STOP\n");
        doCommandWrite("PSTATE\n");
    }

    void CfrClientControl::startSession()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "HANDSHAKING");
    }

    void CfrClientControl::doCommandRead()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Reading command reply");
        boost::asio::async_read_until(command_socket_, command_read_buffer_, DELIMITER_,
                                      std::bind(&CfrClientControl::handleCommandRead,
                                                this, std::placeholders::_1,
                                                std::placeholders::_2));
    }

    void CfrClientControl::doCommandWrite(const std::string& write_message)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Sending to command port: " << write_message);
        boost::asio::async_write(command_socket_, boost::asio::buffer(write_message),
                                 std::bind(&CfrClientControl::handleCommandWrite, this,
                                           std::placeholders::_1, std::placeholders::_2));

        // boost::asio::async_write(
        // command_socket_, boost::asio::buffer(write_message),
        // [&](const boost::system::error_code& ec, std::size_t buffer_size) {
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //                    "LAMDA");
        //     if (!ec) {
        //         std::cout << "lamda handler ok"
        //                   << "\n";
        //     }
        //     else {
        //         std::cout << "lamda handler not ok"
        //                   << "\n ";
        //     }
        // });

        // boost::asio::async_write(
        // command_socket_, boost::asio::buffer(write_message),
        // boost::bind(&CfrClientControl::handleCommandWrite, this,
        //             boost::asio::placeholders::error,
        //             boost::asio::placeholders::bytes_transferred));

        // boost::asio::write(command_socket_, boost::asio::buffer(write_message));

        // boost::asio::read_until(command_socket_, command_read_buffer_, DELIMITER_);
        // std::string stream_read_buffer_str(
        // (std::istreambuf_iterator<char>(&command_read_buffer_)),
        // std::istreambuf_iterator<char>());
        // RCLCPP_INFO_STREAM(this->get_logger(), "Reading feedback buffer: " <<
        // stream_read_buffer_str);
        // command_read_buffer_.consume(command_read_buffer_.size());
        // doCommandRead();
    }

    void CfrClientControl::doFeedbackRead()
    {
        boost::asio::async_read_until(feedback_socket_, feedback_read_buffer_, "\n",
                                      std::bind(&CfrClientControl::handleFeedbackRead,
                                                this, std::placeholders::_1,
                                                std::placeholders::_2));
    }

    void CfrClientControl::handleCommandRead(const boost::system::error_code& ec,
                                             std::size_t bytes_transfered)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Handling command read reply");

        if (!ec) {
            std::string stream_read_buffer_str(
            (std::istreambuf_iterator<char>(&command_read_buffer_)),
            std::istreambuf_iterator<char>());
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Reading command buffer: " << stream_read_buffer_str);
            command_read_buffer_.consume(command_read_buffer_.size());
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Command read failed");
        }
    }

    void CfrClientControl::handleCommandWrite(const boost::system::error_code& ec,
                                              std::size_t bytes_transfered)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Handling command write");
        if (!ec) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Written bytes: " << bytes_transfered);
            doCommandRead();
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Command write failed");
        }
    }

    void CfrClientControl::handleFeedbackRead(const boost::system::error_code& ec,
                                              std::size_t bytes_transfered)
    {
        if (!ec) {
            std::string buffer_str{};
            buffer_str.append((std::istreambuf_iterator<char>(&feedback_read_buffer_)),
                              std::istreambuf_iterator<char>());
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Reading feedback buffer: " << buffer_str);
            feedback_read_buffer_.consume(feedback_read_buffer_.size());
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "feedback read failed");
        }
    }

    void CfrClientControl::twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist_socket_msg_.linear_x_relative = msg->linear.x;
        twist_socket_msg_.linear_y_relative = msg->linear.y;
        twist_socket_msg_.angular_z_relative = msg->angular.z;
    }

    void CfrClientControl::bladeSpeedCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        twist_socket_msg_.blade_speed_rpm = msg->data;
    }

    void CfrClientControl::serverTwistCommandCb()
    {
        auto twist_socket_str = makeStringTwistSocketFormat(twist_socket_msg_);
        RCLCPP_INFO_STREAM(this->get_logger(), "ioc running: " << !ioc_.stopped());
        doCommandWrite(twist_socket_str);
    }

    std::string
    CfrClientControl::makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist)
    {
        std::string retval{"CTRL,"};
        retval.append(std::to_string(input_twist.blade_speed_rpm) + ",");
        retval.append(std::to_string(input_twist.linear_x_relative) + ",");
        retval.append(std::to_string(input_twist.angular_z_relative) + ",");
        retval.append(std::to_string(input_twist.linear_y_relative) + DELIMITER_);
        return retval;
    }

    void CfrClientControl::tokenizeOdom() {}

} // namespace cfr_socket_comm