#include "cfr_socket_comm/cfr_feedback_client.hpp"

namespace cfr_socket_comm {

    CfrFeedbackClient::CfrFeedbackClient(boost::asio::io_context& ioc,
                                         const std::string& host,
                                         const std::string& port)
        : Node("cfr_feedback_client")
        , ioc_(ioc)
        , socket_(ioc_)
        , resolver_(ioc_)
        , odom_pub_(this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10))
        , blade_speed_rpm_pub_(
          this->create_publisher<std_msgs::msg::Float32>("~/blade_speed_rpm", 10))
        , blade_angle_deg_pub_(
          this->create_publisher<std_msgs::msg::Float32>("~/blade_angle_deg", 10))
        , motor_position_deg_pub_(
          this->create_publisher<std_msgs::msg::Float32MultiArray>("~/motor_angle_deg",
                                                                   10))
    {
        try {
            boost::asio::connect(socket_, resolver_.resolve(host, port));
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
    }

    void CfrFeedbackClient::pubCb(const std::string& msg)
    {
        auto result = cfr_socket_comm::ProtocolHandler::tokenizeOdom(msg);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = rclcpp::Clock().now();
        odom_msg.pose.pose.position.x = result.position_x_m;
        odom_msg.pose.pose.position.y = result.position_y_m;
        auto q = ProtocolHandler::EulerToQuaternion(
        {0.0, 0.0, ProtocolHandler::degToRad(result.theta_deg)});
        odom_msg.pose.pose.orientation.x = q.getX();
        odom_msg.pose.pose.orientation.y = q.getY();
        odom_msg.pose.pose.orientation.z = q.getZ();
        odom_msg.pose.pose.orientation.w = q.getW();
        odom_msg.twist.twist.linear.x = result.velocity_linear_x;
        odom_msg.twist.twist.linear.y = result.velocity_linear_y;
        odom_msg.twist.twist.angular.z = result.velocity_theta;

        std_msgs::msg::Float32 blade_speed_rpm_msg;
        blade_speed_rpm_msg.data = result.blade_speed_rpm;

        std_msgs::msg::Float32 blade_angle_deg_msg;
        blade_angle_deg_msg.data = result.blade_angle_deg;

        std_msgs::msg::Float32MultiArray motor_position_deg_msg;
        motor_position_deg_msg.data.push_back(result.LX_motor_angle_deg);
        motor_position_deg_msg.data.push_back(result.RX_motor_angle_deg);
        motor_position_deg_msg.data.push_back(result.RY_motor_angle_deg);

        odom_pub_->publish(odom_msg);
        blade_speed_rpm_pub_->publish(blade_speed_rpm_msg);
        blade_angle_deg_pub_->publish(blade_angle_deg_msg);
        motor_position_deg_pub_->publish(motor_position_deg_msg);
    }

    void CfrFeedbackClient::start()
    {
        std::thread ioc_t([this]() { ioc_.run(); });
        while (rclcpp::ok()) {
            try {
                std::string result;
                boost::asio::streambuf streambuf;
                boost::asio::read_until(socket_, streambuf, "\n");

                std::string stream_read_buffer_str(
                (std::istreambuf_iterator<char>(&streambuf)),
                std::istreambuf_iterator<char>());
                std::cout << stream_read_buffer_str << "\n";
                pubCb(stream_read_buffer_str);
            }
            catch (std::exception& e) {
                std::cerr << "Exception: " << e.what() << "\n";
            }
        }
    }

} // namespace cfr_socket_comm