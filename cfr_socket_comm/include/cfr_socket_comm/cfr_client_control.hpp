#ifndef CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_
#define CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_

#include "cfr_socket_comm/cfr_protocol_handler.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <chrono>
#include <functional>

namespace cfr_socket_comm {
    using boost::asio::ip::tcp;

    class CfrClientControl : public rclcpp::Node {
    public:
        CfrClientControl(const std::string& host,
                         const std::string& command_port,
                         const std::string& feedback_port,
                         boost::asio::io_context& ioc);

        void start();

    private:
        static constexpr int MAX_BUFFER_SIZE{1024};
        static constexpr char DELIMITER_{'\n'};
        std::string host_{};
        std::string command_port_{};
        std::string feedback_port_{};
        boost::asio::io_context& ioc_;
        tcp::socket command_socket_{ioc_};
        tcp::socket feedback_socket_{ioc_};

        CFRTwistSocketFormat twist_socket_msg_;
        CFRFeedbackSocketFormat feedback_socket_msg_;
        boost::asio::streambuf command_read_buffer_;
        boost::asio::streambuf feedback_read_buffer_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr blade_speed_sub_;
        rclcpp::TimerBase::SharedPtr server_command_timer_;
        bool start_control_ {false};

        void twistCb(geometry_msgs::msg::Twist::SharedPtr msg);
        void bladeSpeedCb(std_msgs::msg::Float64::SharedPtr msg);
        void serverTwistCommandCb();

        bool initCommandConnection();
        bool initFeedbackConnection();
        bool initCFREngine();
        void startSession();

        void doCommandWrite(const std::string& write_message);
        void handleCommandWrite(const boost::system::error_code& ec,
                                std::size_t bytes_transfered);

        void doCommandRead();
        void handleCommandRead(const boost::system::error_code& ec,
                               std::size_t bytes_transfered);
        
        void doFeedbackRead();
        void handleFeedbackRead(const boost::system::error_code& ec,
                                std::size_t bytes_transfered);
 
    };
} // namespace cfr_socket_comm

#endif