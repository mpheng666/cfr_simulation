#ifndef CFR_SOCKET_COMM_FEEDBACK_CLIENT_HPP_
#define CFR_SOCKET_COMM_FEEDBACK_CLIENT_HPP_

#include "cfr_socket_comm/cfr_protocol_handler.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>

#include <memory>
#include <chrono>
#include <functional>

namespace cfr_socket_comm {

    using boost::asio::ip::tcp;
    using namespace std::chrono_literals;

    class CfrFeedbackClient : public rclcpp::Node {
    public:
        CfrFeedbackClient(boost::asio::io_context& ioc_, const std::string& host, const std::string& port);
        void start();

    private:
        boost::asio::io_context& ioc_;
        tcp::socket socket_;
        tcp::resolver resolver_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr blade_speed_rpm_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr blade_angle_deg_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_position_deg_pub_;

        void pubCb(const std::string& msg);

    };
} // namespace cfr_socket_comm

#endif