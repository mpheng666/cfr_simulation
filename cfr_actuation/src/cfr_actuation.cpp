#include "cfr_actuation/cfr_actuation.hpp"

namespace cfr_actuation_ns
{
    CfrActuation::CfrActuation():
    Node("cfr_actuation"),
    actuation_pub_(this->create_publisher<std_msgs::msg::Float64MultiArray>("cfr_actuation", 10)),
    joy_sub_(this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CfrActuation::joyCb, this, _1))),
    allow_move_sub_(this->create_subscription<std_msgs::msg::Bool>("allow_move", 10, std::bind(&CfrActuation::allowMoveCb, this, _1))),
    timer_(this->create_wall_timer(10ms, std::bind(&CfrActuation::timerCb, this))),
    reset_timer_(this->create_wall_timer(2s, std::bind(&CfrActuation::resetTimerCb, this)))
    {
        this->loadParams();
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

    void CfrActuation::resetTimerCb()
    {
        reset_flag_ = false;
    }

    void CfrActuation::allowMoveCb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        allow_move_ = msg->data;
    }

    void CfrActuation::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if(allow_move_)
        {
            double JoystickLeftX = joy_remapper_.joy_left_x_magnitude * msg->axes.at(joy_remapper_.joy_left_x_axis);
            double JoystickLeftY = joy_remapper_.joy_left_y_magnitude * msg->axes.at(joy_remapper_.joy_left_y_axis); 
            double JoystickRightX = joy_remapper_.joy_right_x_magnitude * msg->axes.at(joy_remapper_.joy_right_x_axis);
            double JoystickRightY = joy_remapper_.joy_right_y_magnitude * msg->axes.at(joy_remapper_.joy_right_y_axis);
            // RCLCPP_INFO(this->get_logger(), "JoystickLeftX: %f", JoystickLeftX);
            // RCLCPP_INFO(this->get_logger(), "JoystickLeftY: %f", JoystickLeftY);
            // RCLCPP_INFO(this->get_logger(), "JoystickRightX: %f", JoystickRightX);
            // RCLCPP_INFO(this->get_logger(), "JoystickRightY: %f", JoystickRightY);
            this->joyToMotorActuation(JoystickLeftX, JoystickLeftY, JoystickRightX, JoystickRightY);
        }
        else if(reset_flag_ == false 
        && (msg->axes.at(joy_remapper_.joy_left_x_axis) > 0.0 
        || msg->axes.at(joy_remapper_.joy_right_x_axis) > 0.0 
        || msg->axes.at(joy_remapper_.joy_right_y_axis) > 0.0))
        {
            // system("notify-send -u low --hint int:transient:1 'CFR SIMULATION' 'Please run the blades before moving!'");
            reset_flag_ = true;
            this->joyToMotorActuation(0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            this->joyToMotorActuation(0.0, 0.0, 0.0, 0.0);
        }
    }

    void CfrActuation::loadParams()
    {
        motor_deg_limit_.LXmin = this->declare_parameter("LXmin", -16.5);
        motor_deg_limit_.LXmax = this->declare_parameter("LXmax", 16.5);
        motor_deg_limit_.RXmin = this->declare_parameter("RXmin", -16.5);
        motor_deg_limit_.RXmax = this->declare_parameter("RXmax", 16.5);
        motor_deg_limit_.RYmin = this->declare_parameter("RYmin", -12.5);
        motor_deg_limit_.RYmax = this->declare_parameter("RYmax", 12.5);

        joy_remapper_.joy_left_x_axis = this->declare_parameter("joy_left_x_axis", 0);
        joy_remapper_.joy_left_y_axis = this->declare_parameter("joy_left_y_axis", 1);
        joy_remapper_.joy_right_x_axis = this->declare_parameter("joy_right_x_axis", 2);
        joy_remapper_.joy_right_y_axis = this->declare_parameter("joy_right_y_axis", 3);

        joy_remapper_.joy_left_x_magnitude = this->declare_parameter("joy_left_x_magnitude", 1.0);
        joy_remapper_.joy_left_y_magnitude = this->declare_parameter("joy_left_y_magnitude", 1.0);
        joy_remapper_.joy_right_x_magnitude = this->declare_parameter("joy_right_x_magnitude", 1.0);
        joy_remapper_.joy_right_y_magnitude = this->declare_parameter("joy_right_y_magnitude", 1.0);
    }

    void CfrActuation::joyToMotorActuation(const double JoystickLeftX, [[maybe_unused]] const double JoystickLeftY, const double JoystickRightX, const double JoystickRightY)
    {
        motor_actuation_.LXMotordeg_FB = motor_deg_limit_.LXmin*(1-(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.LXmax*(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin); 
        motor_actuation_.RXMotordeg_FB = -(motor_deg_limit_.RXmin*(1-(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RXmax*(JoystickRightY-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin));

        motor_actuation_.LXMotordeg_ROT = -(motor_deg_limit_.LXmin*(1-(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.LXmax*(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)); 
        motor_actuation_.RXMotordeg_ROT = -(motor_deg_limit_.RXmin*(1-(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RXmax*(JoystickRightX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin));

        motor_actuation_.LXMotordeg = motor_actuation_.LXMotordeg_FB + motor_actuation_.LXMotordeg_ROT;
        motor_actuation_.RXMotordeg = motor_actuation_.RXMotordeg_FB + motor_actuation_.RXMotordeg_ROT;

        motor_actuation_.RYMotordeg = motor_deg_limit_.RYmin*(1-(JoystickLeftX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin)) + motor_deg_limit_.RYmax*(JoystickLeftX-joy_limit_.joymin)/(joy_limit_.joymax-joy_limit_.joymin);

        // [[maybe_unused]] double dummy = JoystickLeftY;

        // motor_actuation_.print();
    }

} // cfr_actuation_ns