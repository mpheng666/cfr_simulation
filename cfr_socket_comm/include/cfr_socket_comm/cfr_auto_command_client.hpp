#ifndef CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_
#define CFR_SOCKET_COMM_CFR_CLIENT_CONTROL_HPP_

#include "cfr_socket_comm/cfr_protocol_handler.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <chrono>
#include <functional>

namespace cfr_socket_comm {
    using boost::asio::ip::tcp;

    class CfrAutoCommandClient : public rclcpp::Node {
    public:
        CfrAutoCommandClient(const std::string& host,
                             const std::string& command_port,
                             boost::asio::io_context& ioc);

        void start();

    private:
        static constexpr int MAX_BUFFER_SIZE{1024};
        static constexpr char DELIMITER_{'\n'};
        std::string host_{};
        std::string command_port_{};
        boost::asio::io_context& ioc_;
        tcp::socket command_socket_{ioc_};
        tcp::socket feedback_socket_{ioc_};

        bool start_cfr_twist_{false};

        CFRTwistSocketFormat twist_socket_msg_;

        // std::vector<std::string> commands_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr blade_speed_sub_;

        std::vector<std::string> commands_{
        "PSTATE", "MODE",       "BEACONS",     "NYPAUTO,1", "INIT",
        "PSTATE", "PSTATE",     "BLADEANG,10", "FB,1", "START", 
        "PSTATE", "CTRL,0,0,0,0", "PSTATE"};

        void twistCb(geometry_msgs::msg::Twist::SharedPtr msg);
        void bladeSpeedCb(std_msgs::msg::Int32::SharedPtr msg);
        bool initCommandConnection();
        void loadCommands();
        void startSession();
        bool verifyReplyCommand(const std::string& command, const std::string& reply);

        void doCommandWrite(const std::string& write_message);
        std::string doCommandRead();
    };
} // namespace cfr_socket_comm

#endif
