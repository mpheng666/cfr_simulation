#ifndef CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_
#define CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_

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

    struct CFRFeedbackSocketFormat {
        uint32_t timestamped_ms{0};
        double position_x_m{0.0};
        double position_y_m{0.0};
        double theta_deg{0.0};
        double blade_speed_rpm{0.0};
        double blade_angle_deg{0.0};
        double velocity_linear_x{0.0};
        double velocity_linear_y{0.0};
        double velocity_theta{0.0};
        double LX_motor_angle_deg{0.0};
        double RX_motor_angle_deg{0.0};
        double RY_motor_angle_deg{0.0};
    };

    struct CFRTwistSocketFormat {
        double blade_speed_rpm{0.0};
        double linear_x_relative{0.0};
        double angular_z_relative{0.0};
        double linear_y_relative{0.0};
    };

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

        void twistCb(geometry_msgs::msg::Twist::SharedPtr msg);
        void bladeSpeedCb(std_msgs::msg::Float64::SharedPtr msg);
        void serverTwistCommandCb();

        bool initCommandConnection();
        bool initFeedbackConnection();
        bool initCFREngine();
        void startSession();

        void doCommandRead();
        void doCommandWrite(const std::string& write_message);
        void doFeedbackRead();

        void handleCommandRead(const boost::system::error_code& ec,
                               std::size_t bytes_transfered);
        void handleCommandWrite(const boost::system::error_code& ec,
                                std::size_t bytes_transfered);
        void handleFeedbackRead(const boost::system::error_code& ec,
                                std::size_t bytes_transfered);

        std::string makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist);

        void tokenizeOdom();
    };
} // namespace cfr_socket_comm

#endif