#ifndef CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_
#define CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <chrono>

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
        double angualr_z_relative{0.0};
        double linear_y_relative{0.0};
    };

    class CfrClientControl : public rclcpp::Node {
    public:
        CfrClientControl(const std::string& host,
                         const std::string& command_port,
                         const std::string& feedback_port);

        void start();

    private:
        boost::asio::io_context ioc_;
        tcp::socket command_socket_{ioc_};
        tcp::socket feedback_socket_{ioc_};
        static constexpr int MAX_BUFFER_SIZE{1024};
        std::string host_{};
        std::string command_port_{};
        std::string feedback_port_{};

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        // rclcpp::TimerBase::SharedPtr twist_pub_timer_;

        void twistCb(const geometry_msgs::msg::Twist::ConstPtr msg);

        bool initCommandConnection();
        bool initFeedbackConnection();
        void startSession();

        void doCommandRead(const std::string& read_message);
        void doCommandWrite(const std::string& write_message);
        void doFeedbackRead(const std::string& read_message);

        void handleCommandRead(boost::system::error_code& ec, size_t bytes_transfered);
        void handleCommandWrite(boost::system::error_code& ec, size_t bytes_transfered);
        void handleFeedbackRead(boost::system::error_code& ec, size_t bytes_transfered);

        void decodeFeedback();
        void publishFeedback();

        CFRTwistSocketFormat encodeTwistToSocketFormat(const geometry_msgs::msg::Twist::ConstPtr& msg);
        std::string makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist);
    };
} // namespace cfr_socket_comm

#endif