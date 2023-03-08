#include "cfr_socket_comm/cfr_client_control.hpp"

namespace cfr_socket_comm {
    CfrClientControl::CfrClientControl(const std::string& host,
                                       const std::string& command_port,
                                       const std::string& feedback_port)
        : Node("cfr_client_control")
        , host_(host)
        , command_port_(command_port)
        , feedback_port_(feedback_port)
        , odom_pub_(this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10))
        , twist_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "~/cmd_vel",
          10,
          std::bind(&CfrClientControl::twistCb, this, std::placeholders::_1)))
    {
    }

    void CfrClientControl::start()
    {
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
    }

    void CfrClientControl::startSession() {}

    void CfrClientControl::doCommandRead(const std::string& read_message)
    {
        boost::asio::streambuf stream_read_buffer;

        boost::asio::async_read_until(
        command_socket_, stream_read_buffer, "\n",
        boost::bind(&CfrClientControl::handleCommandRead,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
    }

    void CfrClientControl::doCommandWrite(const std::string& write_message)
    {
        boost::asio::async_write(
        command_socket_, boost::asio::buffer(write_message),
        boost::bind(&CfrClientControl::handleCommandWrite,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
    }

    void CfrClientControl::doFeedbackRead(const std::string& read_message)
    {
        boost::asio::streambuf stream_read_buffer;
        std::string buffer_str{};
        try {
            boost::asio::read_until(feedback_socket_, stream_read_buffer, "\n");
            buffer_str.append((std::istreambuf_iterator<char>(&stream_read_buffer)),
                              std::istreambuf_iterator<char>());
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
        }
    }

    void CfrClientControl::handleCommandRead(boost::system::error_code& ec,
                                             size_t bytes_transfered)
    {
        if (!ec) {
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Command read failed");
        }
    }

    void CfrClientControl::handleCommandWrite(boost::system::error_code& ec,
                                              size_t bytes_transfered)
    {
        if (!ec) {
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "Command write failed");
        }
    }

    void CfrClientControl::handleFeedbackRead(boost::system::error_code& ec,
                                              size_t bytes_transfered)
    {
        if (!ec) {
        }
        else {
            RCLCPP_WARN_STREAM(this->get_logger(), "feedback read failed");
        }
    }

    void CfrClientControl::twistCb(const geometry_msgs::msg::Twist::ConstPtr msg) {
        auto twist_socket = encodeTwistToSocketFormat(msg);
        auto twist_socket_str = makeStringTwistSocketFormat(twist_socket);
        doCommandWrite(twist_socket_str);
    }

    CFRTwistSocketFormat CfrClientControl::encodeTwistToSocketFormat(const geometry_msgs::msg::Twist::ConstPtr& msg)
    {

    }

    std::string CfrClientControl::makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist)
    {

    }

} // namespace cfr_socket_comm