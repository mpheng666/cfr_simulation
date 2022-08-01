#include "cfr_dynamics/cfr_dynamics.hpp"
namespace cfr_dynamics_ns
{
    CfrDynamics::CfrDynamics():
    Node("cfr_dynamics"),
    accel_pub_(this->create_publisher<geometry_msgs::msg::Accel>("cfr_acceleration", 10)),
    actuation_sub_(this->create_subscription<std_msgs::msg::Float64MultiArray>("cfr_actuation", 10, std::bind(&CfrDynamics::actuationCb, this, _1))),
    cmd_vel_sub_(this->create_subscription<geometry_msgs::msg::Twist>("cfr/cmd_vel", 10, std::bind(&CfrDynamics::cmdVelCb, this, _1))),
    odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&CfrDynamics::odomCb, this, _1))),
    timer_(this->create_wall_timer(10ms, std::bind(&CfrDynamics::timerCb, this)))
    {
        this->loadParams();
    }

    CfrDynamics::~CfrDynamics()
    {
        
    }

    void CfrDynamics::timerCb()
    {
        auto accel_msg = geometry_msgs::msg::Accel();
        accel_msg.linear.x = dynamics_gen_.x_acc;
        accel_msg.linear.y = dynamics_gen_.y_acc;
        accel_msg.angular.z = dynamics_gen_.theta_acc;
        accel_pub_->publish(accel_msg);
    }

    void CfrDynamics::actuationCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        model_inputs_.LXMotordeg = msg->data.at(0);
        model_inputs_.RXMotordeg = msg->data.at(1);
        model_inputs_.RYMotordeg = msg->data.at(2);
        // model_inputs_.print();
        // model_inputs_.NetTorque = msg->data.at(3);
        model_inputs_.NetTorque = 45.0;
        this->computeEffectiveContactRadius();
        this->computeDynamics();
    }

    void CfrDynamics::cmdVelCb([[maybe_unused]] const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // model_inputs_.vx = msg->linear.x;
        // model_inputs_.vy = msg->linear.y;
        // model_inputs_.theta_dot = msg->angular.z;
    }

    void CfrDynamics::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        model_inputs_.vx = msg->twist.twist.linear.x;
        model_inputs_.vy = msg->twist.twist.linear.y;
        model_inputs_.theta_dot = msg->twist.twist.angular.z;
    }

    void CfrDynamics::loadParams()
    {

    }

    void CfrDynamics::computeEffectiveContactRadius()
    {
        // left X
        effective_c_r_.k_left_x = (cfr_model_.blade_rad_max - cfr_model_.blade_rad_min)/ (cfr_model_.motors_limits.LXmin_abs - cfr_model_.motors_limits.LXmax);
        effective_c_r_.b_left_x = cfr_model_.blade_rad_max - effective_c_r_.k_left_x * cfr_model_.motors_limits.LXmin_abs;

        if(abs(model_inputs_.LXMotordeg) < cfr_model_.motors_limits.LXmin_abs)
            effective_c_r_.effective_rad_LX = std::numeric_limits<double>::infinity();
        else
            effective_c_r_.effective_rad_LX = effective_c_r_.k_left_x * abs(model_inputs_.LXMotordeg) + effective_c_r_.b_left_x;

        // right X
        effective_c_r_.k_right_x = (cfr_model_.blade_rad_max - cfr_model_.blade_rad_min)/ (cfr_model_.motors_limits.RXmin_abs - cfr_model_.motors_limits.RXmax);
        effective_c_r_.b_right_x = cfr_model_.blade_rad_max - effective_c_r_.k_right_x * cfr_model_.motors_limits.RXmin_abs;

        if(abs(model_inputs_.RXMotordeg) < cfr_model_.motors_limits.RXmin_abs)
            effective_c_r_.effective_rad_RX = std::numeric_limits<double>::infinity();
        else
            effective_c_r_.effective_rad_RX = effective_c_r_.k_right_x * abs(model_inputs_.RXMotordeg) + effective_c_r_.b_right_x;

        // right Y
        effective_c_r_.k_right_y = (cfr_model_.blade_rad_max - cfr_model_.blade_rad_min) / (cfr_model_.motors_limits.RYmin_abs - cfr_model_.motors_limits.RYmax);
        effective_c_r_.b_right_y = cfr_model_.blade_rad_max - effective_c_r_.k_right_y * cfr_model_.motors_limits.RYmin_abs;
        if(abs(model_inputs_.RYMotordeg) <  cfr_model_.motors_limits.RYmin_abs)
            effective_c_r_.effective_rad_RY = std::numeric_limits<double>::infinity();
        else
        {
            if(model_inputs_.RYMotordeg >= 0)
                effective_c_r_.effective_rad_RY = effective_c_r_.k_right_y * abs(model_inputs_.RYMotordeg) + effective_c_r_.b_right_y;
            else
                effective_c_r_.effective_rad_RY = -(effective_c_r_.k_right_y * abs(model_inputs_.RYMotordeg) + effective_c_r_.b_right_y);
        }
        effective_c_r_.print();
    }

    void CfrDynamics::computeDynamics()
    {
        // compute force generated by left rotor in y direction
        dynamics_gen_.Fy_l_rotor = model_params_.k_left * model_inputs_.NetTorque / effective_c_r_.effective_rad_LX;
        if(model_inputs_.LXMotordeg > 0)
            dynamics_gen_.Fy_l_rotor = dynamics_gen_.Fy_l_rotor;
        else
            dynamics_gen_.Fy_l_rotor = -dynamics_gen_.Fy_l_rotor;

        // compute force generated by right rotor in y direction
        dynamics_gen_.Fy_r_rotor = model_params_.k_right * model_inputs_.NetTorque / effective_c_r_.effective_rad_RX;
        if(model_inputs_.RXMotordeg > 0)
            dynamics_gen_.Fy_r_rotor = -dynamics_gen_.Fy_r_rotor;
        else
            dynamics_gen_.Fy_r_rotor = dynamics_gen_.Fy_r_rotor;

        dynamics_gen_.Fy_rotor = dynamics_gen_.Fy_l_rotor + dynamics_gen_.Fy_r_rotor;

        // friction force
        dynamics_gen_.F_friction = environment_params_.mu_1 * cfr_model_.mass * environment_params_.g;
        // drag force
        dynamics_gen_.F_drag_y = environment_params_.mu_3 * model_inputs_.vy;
        dynamics_gen_.Fy =  dynamics_gen_.Fy_rotor - dynamics_gen_.F_friction - dynamics_gen_.F_drag_y; // combined force in y direction
        dynamics_gen_.y_acc = dynamics_gen_.Fy / cfr_model_.mass; // acceleration in y direction

        // compute force generated by right rotor in x direction
        // dynamics_gen_.Fx_r_rotor = model_params_.k_right * model_inputs_.NetTorque / effective_c_r_.effective_rad_RY - 61.967213;
        dynamics_gen_.Fx_r_rotor = model_params_.k_right * model_inputs_.NetTorque / effective_c_r_.effective_rad_RY;
        dynamics_gen_.F_drag_x = environment_params_.mu_3 * model_inputs_.vx;
        dynamics_gen_.Fx = dynamics_gen_.Fx_r_rotor - dynamics_gen_.F_friction - dynamics_gen_.F_drag_x;
        dynamics_gen_.x_acc = dynamics_gen_.Fx / cfr_model_.mass;

        if(effective_c_r_.effective_rad_LX != std::numeric_limits<double>::infinity())
        {
            if(model_inputs_.LXMotordeg > 0)
                dynamics_gen_.L_lever = cfr_model_.width/2 + effective_c_r_.effective_rad_LX;
            else
                dynamics_gen_.L_lever = cfr_model_.width/2 - effective_c_r_.effective_rad_LX;
        }
        else
            dynamics_gen_.L_lever = 0.0;

        if(effective_c_r_.effective_rad_RX != std::numeric_limits<double>::infinity())
        {
            if(model_inputs_.RXMotordeg > 0)
                dynamics_gen_.R_lever = cfr_model_.width/2 - effective_c_r_.effective_rad_RX;
            else
                dynamics_gen_.R_lever = cfr_model_.width/2 + effective_c_r_.effective_rad_RX;  
        }
        else
            dynamics_gen_.R_lever = 0.0;

        dynamics_gen_.rot_torque_left = dynamics_gen_.Fy_l_rotor * dynamics_gen_.L_lever;
        dynamics_gen_.rot_torque_right = dynamics_gen_.Fy_r_rotor * dynamics_gen_.R_lever;


        if(dynamics_gen_.rot_torque_left <= 0)
            dynamics_gen_.theta_acc_left =  dynamics_gen_.rot_torque_left/cfr_model_.I;  // CW
        else
            dynamics_gen_.theta_acc_left =  dynamics_gen_.rot_torque_left/cfr_model_.I;  // CCW

        if(dynamics_gen_.rot_torque_right <= 0)
            dynamics_gen_.theta_acc_right =  -dynamics_gen_.rot_torque_right/cfr_model_.I;  // CCW
        else
            dynamics_gen_.theta_acc_right =  -dynamics_gen_.rot_torque_right/cfr_model_.I;  // CW

        dynamics_gen_.theta_acc = dynamics_gen_.theta_acc_left + dynamics_gen_.theta_acc_right  - (environment_params_.mu_4 * model_inputs_.theta_dot)/cfr_model_.I;

        // dynamics_gen_.print();
    } 

} // cfr_dynamics_ns