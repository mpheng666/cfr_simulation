#include "cfr_actuation/cfr_actuation.hpp"

namespace cfr_actuation_ns
{
    CfrActuation::CfrActuation():
    Node("cfr_actuation"),
    actuation_pub_(this->create_publisher<std_msgs::msg::Float64MultiArray>("cfr_actuation", 10)),
    joy_sub_(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CfrActuation::joyCb, this, _1))),
    timer_(this->create_wall_timer(10ms, std::bind(&CfrActuation::timerCb, this)))
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
        actuation_msg.data.at(0) = motor_actuation_.LXMotordeg;
        actuation_msg.data.at(1) = motor_actuation_.RXMotordeg;
        actuation_msg.data.at(2) = motor_actuation_.RYMotordeg;
        actuation_pub_->publish(actuation_msg);
    }

    void CfrActuation::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        double JoystickLeftX = msg->axes.at(0);
        double JoystickLeftY = msg->axes.at(1); 
        double JoystickRightX = msg->axes.at(2);
        double JoystickRightY = msg->axes.at(3);
        // RCLCPP_INFO(this->get_logger(), "JoystickLeftX: %f", JoystickLeftX);
        // RCLCPP_INFO(this->get_logger(), "JoystickLeftY: %f", JoystickLeftY);
        // RCLCPP_INFO(this->get_logger(), "JoystickRightX: %f", JoystickRightX);
        // RCLCPP_INFO(this->get_logger(), "JoystickRightY: %f", JoystickRightY);

        this->joyToMotorActuation(JoystickLeftX, JoystickLeftY, JoystickRightX, JoystickRightY);
    }

    void CfrActuation::loadParams()
    {
        motor_deg_limit_.LXmin = this->declare_parameter("LXmin", -16.5);
        motor_deg_limit_.LXmax = this->declare_parameter("LXmax", 16.5);
        motor_deg_limit_.RXmin = this->declare_parameter("RXmin", -16.5);
        motor_deg_limit_.RXmax = this->declare_parameter("RXmax", 16.5);
        motor_deg_limit_.RYmin = this->declare_parameter("RYmin", -12.5);
        motor_deg_limit_.RYmax = this->declare_parameter("RYmax", 12.5);
    }

    void CfrActuation::joyToMotorActuation(const double JoystickLeftX, const double JoystickLeftY, const double JoystickRightX, const double JoystickRightY)
    {
        motor_actuation_.LXMotordeg_FB = motor_deg_limit_.LXmin*(1-(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.LXmax*(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin); 
        motor_actuation_.RXMotordeg_FB = -(motor_deg_limit_.RXmin*(1-(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RXmax*(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin));

        motor_actuation_.LXMotordeg_ROT = -(motor_deg_limit_.LXmin*(1-(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.LXmax*(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)); 
        motor_actuation_.RXMotordeg_ROT = -(motor_deg_limit_.RXmin*(1-(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RXmax*(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin));

        motor_actuation_.LXMotordeg = motor_actuation_.LXMotordeg_FB + motor_actuation_.LXMotordeg_ROT;
        motor_actuation_.RXMotordeg = motor_actuation_.RXMotordeg_FB + motor_actuation_.RXMotordeg_ROT;

        motor_actuation_.RYMotordeg = motor_deg_limit_.RYmin*(1-(JoystickLeftX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RYmax*(JoystickLeftX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin); 

        motor_actuation_.print();
    }

} // cfr_actuation_ns