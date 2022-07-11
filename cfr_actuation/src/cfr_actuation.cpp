#include "cfr_actuation/cfr_actuation.hpp"

namespace cfr_actuation_ns
{
    CfrActuation::CfrActuation():
    Node("cfr_actuation"),
    actuation_pub_(this->create_publisher<std_msgs::msg::Float64MultiArray>("cfr_actuation", 10)),
    joy_sub_(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CfrActuation::joyCb, this, _1))),
    timer_(this->create_wall_timer(100ms, std::bind(&CfrActuation::timerCb, this)))
    {
        this->loadParams();
    }

    CfrActuation::~CfrActuation()
    {
        
    }

    void CfrActuation::timerCb()
    {
        auto actuation_msg = std_msgs::msg::Float64MultiArray();
        actuation_msg.data.resize(3);
        actuation_msg.data.at(0) = motor_actuation_.left_x_motor_deg;
        actuation_msg.data.at(1) = motor_actuation_.right_x_motor_deg;
        actuation_msg.data.at(2) = motor_actuation_.right_y_motor_deg;
        actuation_pub_->publish(actuation_msg);
    }

    void CfrActuation::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        double joy_left_x = msg->axes.at(0);
        double joy_left_y = msg->axes.at(1); 
        double joy_right_x = msg->axes.at(2);
        double joy_right_y = msg->axes.at(3);
        // RCLCPP_INFO(this->get_logger(), "joy_left_x: %f", joy_left_x);
        // RCLCPP_INFO(this->get_logger(), "joy_left_y: %f", joy_left_y);
        // RCLCPP_INFO(this->get_logger(), "joy_right_x: %f", joy_right_x);
        // RCLCPP_INFO(this->get_logger(), "joy_right_y: %f", joy_right_y);

        this->joyToMotorActuation(joy_left_x, joy_left_y, joy_right_x, joy_right_y);
    }

    void CfrActuation::loadParams()
    {
        left_x_min_ = this->declare_parameter("left_x_min", -1.0);
        left_x_max_ = this->declare_parameter("left_x_max", 1.0);
        right_x_min_ = this->declare_parameter("right_x_min", -1.0);
        right_x_max_ = this->declare_parameter("right_x_max", 1.0);
        right_y_min_ = this->declare_parameter("right_y_min", -1.0);
        right_y_max_ = this->declare_parameter("right_y_max", 1.0);
    }

    void CfrActuation::joyToMotorActuation(const double joy_left_x, const double joy_left_y, const double joy_right_x, const double joy_right_y)
    {
        motor_actuation_.left_x_motor_deg_fb =  left_x_min_*joy_right_y + left_x_max_*joy_right_y;
        motor_actuation_.right_x_motor_deg_fb = right_x_min_*joy_right_y + right_x_max_*joy_right_y;
        motor_actuation_.left_x_motor_deg_rot = -left_x_min_*joy_right_x + left_x_max_*joy_right_x;
        motor_actuation_.right_x_motor_deg_rot = -right_x_min_*joy_right_x + right_x_max_*joy_right_x;
        motor_actuation_.left_x_motor_deg = motor_actuation_.left_x_motor_deg_fb + motor_actuation_.left_x_motor_deg_rot;
        motor_actuation_.right_x_motor_deg = motor_actuation_.right_x_motor_deg_fb + motor_actuation_.right_x_motor_deg_rot;
        motor_actuation_.right_y_motor_deg = right_y_min_*joy_left_x + right_y_max_*joy_left_x;

        motor_actuation_.print();
    }

} // cfr_actuation_ns