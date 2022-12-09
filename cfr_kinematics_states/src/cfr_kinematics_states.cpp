#include "cfr_kinematics_states/cfr_kinematics_states.hpp"

namespace cfr_kinemtics_states_ns {
    CfrKinematicsStates::CfrKinematicsStates():
    Node("cfr_states"),
    cmd_vel_pub_(this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)),
    acc_sub_(this->create_subscription<geometry_msgs::msg::Accel>("cfr_acceleration", 10, std::bind(&CfrKinematicsStates::accelCb, this, _1))),
    timer_(this->create_wall_timer(100ms, std::bind(&CfrKinematicsStates::timerCb, this)))
    {
        this->loadParams();
    }

    void CfrKinematicsStates::loadParams()
    {

    }

    void CfrKinematicsStates::timerCb()
    {
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = kinematics_states_.velocity_linear_x_curr;
        cmd_vel_msg.linear.y = kinematics_states_.velocity_linear_y_curr;
        cmd_vel_msg.angular.z = kinematics_states_.velocity_angular_z_curr;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void CfrKinematicsStates::accelCb(const geometry_msgs::msg::Accel::SharedPtr msg)
    {
        kinematics_states_.acc_linear_x_curr = msg->linear.x;
        kinematics_states_.acc_linear_y_curr = msg->linear.y;
        kinematics_states_.acc_angular_z_curr = msg->angular.z;
        this->updateStates();
    }

    void CfrKinematicsStates::updateStates()
    {
        auto t_ros = rclcpp::Clock();
        t_ = t_ros.now().seconds();
        double delta_t = t_ - t_prev_;
        
        kinematics_states_.velocity_linear_x_curr = 
        kinematics_states_.velocity_linear_x_prev + kinematics_states_.acc_linear_x_curr * delta_t; 
        kinematics_states_.velocity_linear_y_curr = 
        kinematics_states_.velocity_linear_y_prev + kinematics_states_.acc_linear_y_curr * delta_t; 
        kinematics_states_.velocity_angular_z_curr = 
        kinematics_states_.velocity_angular_z_prev + kinematics_states_.acc_angular_z_curr * delta_t; 

        RCLCPP_INFO(this->get_logger(), "Currnet vel linear x: %lf", kinematics_states_.velocity_linear_x_curr);
        RCLCPP_INFO(this->get_logger(), "Currnet vel linear y: %lf", kinematics_states_.velocity_linear_y_curr);
        RCLCPP_INFO(this->get_logger(), "Currnet vel angular z: %lf", kinematics_states_.velocity_angular_z_curr);
        
        kinematics_states_.velocity_linear_x_prev = kinematics_states_.velocity_linear_x_curr;
        kinematics_states_.velocity_linear_y_prev = kinematics_states_.velocity_linear_y_curr;
        kinematics_states_.velocity_angular_z_prev = kinematics_states_.velocity_angular_z_curr;

        t_prev_ = t_;
    }


} // cfr_states_ns