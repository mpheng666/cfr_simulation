#ifndef _CFR_DYNAMICS_CFR_DYNAMICS_HPP_
#define _CFR_DYNAMICS_CFR_DYNAMICS_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_dynamics_ns
{
    struct MotorsLimits
    {
        double LXmin {-16.5}; 
        double LXmax {16.5};
        double RXmin {-16.5};
        double RXmax {16.5};
        double RYmin {-12.5};
        double RYmax {12.5};
        double LXmin_abs {1.0};        
        double RXmin_abs {1.0};        
        double RYmin_abs {0.5}; 
    };

    struct CfrPhysicalProperties
    {
        double mass {550.0};
        double width {2.02};
        double depth {1.05};
        double height {1.6};
        double I {1.0/12.0 * mass * (width*width + depth*depth)};
        double blade_rad_max {0.5};
        double blade_rad_min {0.3};
        MotorsLimits motors_limits;
    };

    struct EnvParams
    {
        double mu_1 {0.0}; // friction coefficient
        // double mu_2 {0.2}; // viscocoty coefficient
        double mu_3 {150.0}; // air drag coefficient
        double mu_4 {400.0}; // rotational resistance coefficient
        double g {9.81};
    };

    struct ModelParams
    {
        double k_left {0.7}; // left rotor damping coefficient
        double k_right {0.7}; // right rotor damping coefficient
        double hy_left {0.02}; // gain of left blade effective radius in y direction
        double hy_right {0.02}; // gain of right blade effective radius in y direction
        double hx_right {0.02}; // gain of right blade effective radius in x direction
    };

    struct ModelInputs
    {
        double LXMotordeg {0.0};
        double RXMotordeg {0.0};
        double RYMotordeg {0.0};
        double NetTorque {0.0};
        double Motordeglimit {0.0};
        double vx {0.0};
        double vy {0.0};
        double theta_dot {0.0};

        void print()
        {
            printf("LXMotordeg: %lf \n", LXMotordeg);
            printf("RXMotordeg: %lf \n", RXMotordeg);
            printf("RYMotordeg: %lf \n", RYMotordeg);    
        }
    };

    struct BladesEffectiveContactRadius
    {
        double k_left_x {0.0};
        double b_left_x {0.0};
        double effective_rad_LX {0.0};
        double k_right_x {0.0};
        double b_right_x {0.0};
        double effective_rad_RX {0.0};
        double k_right_y {0.0};
        double b_right_y {0.0};
        double effective_rad_RY {0.0};

        void print()
        {
            // printf("k_left_x: %lf \n", k_left_x);
            // printf("b_left_x: %lf \n", b_left_x);
            // printf("effective_rad_LX: %lf \n", effective_rad_LX);
            // printf("k_right_x: %lf \n", k_right_x);
            // printf("b_right_x: %lf \n", b_right_x);
            // printf("effective_rad_RX: %lf \n", effective_rad_RX);
            // printf("k_right_y: %lf \n", k_right_y);
            // printf("b_right_y: %lf \n", b_right_y);
            printf("effective_rad_RY: %lf \n", effective_rad_RY);
        };
    };

    struct DynamicsGenerated
    {
        double Fy_l_rotor {0.0};
        double Fy_r_rotor {0.0};
        double Fy_rotor {0.0};
        double F_friction {0.0};
        double F_drag_y {0.0};
        double Fy {0.0};
        double y_acc {0.0};
        double Fx_r_rotor {0.0};
        double F_drag_x {0.0};
        double Fx {0.0};
        double x_acc {0.0};
        double L_lever {0.0};
        double R_lever {0.0};
        double rot_torque_left {0.0};
        double rot_torque_right {0.0};
        double theta_acc_left {0.0};
        double theta_acc_right {0.0};
        double theta_acc {0.0};

        void print()
        {
            // printf("Fy_l_rotor: %lf \n", Fy_l_rotor);
            // printf("Fy_r_rotor: %lf \n", Fy_r_rotor);
            // printf("Fy_rotor: %lf \n", Fy_rotor);
            // printf("F_friction: %lf \n", F_friction);
            // printf("F_drag_y: %lf \n", F_drag_y);
            // printf("Fy: %lf \n", Fy);
            // printf("y_acc: %lf \n", y_acc);
            // printf("Fx_r_rotor: %lf \n", Fx_r_rotor);
            // printf("F_drag_x: %lf \n", F_drag_x);
            // printf("Fx: %lf \n", Fx);
            // printf("x_acc: %lf \n", x_acc);
            // printf("L_lever: %lf \n", L_lever);
            // printf("R_lever: %lf \n", R_lever);
            // printf("rot_torque_left: %lf \n", rot_torque_left);
            // printf("rot_torque_right: %lf \n", rot_torque_right);
            printf("theta_acc_left: %lf \n", theta_acc_left);
            printf("theta_acc_right: %lf \n", theta_acc_right);
            // printf("theta_acc: %lf \n", theta_acc);
        };
    };

    class CfrDynamics : public rclcpp::Node
    {
        public:
            CfrDynamics();
            ~CfrDynamics();

        private:
            CfrPhysicalProperties cfr_model_;
            EnvParams environment_params_;
            ModelParams model_params_;
            ModelInputs model_inputs_;
            BladesEffectiveContactRadius effective_c_r_;
            DynamicsGenerated dynamics_gen_;

            rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_sub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::TimerBase::SharedPtr timer_;

            void loadParams();
            void timerCb();
            void actuationCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
            void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
            void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
            void computeEffectiveContactRadius();
            void computeDynamics();            
    };

} // cfr_dynamics_ns

#endif