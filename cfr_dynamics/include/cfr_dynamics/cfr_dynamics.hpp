#ifndef _CFR_DYNAMICS_CFR_DYNAMICS_HPP_
#define _CFR_DYNAMICS_CFR_DYNAMICS_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace cfr_dynamics_ns {
    struct MotorsLimits {
        double LXmin{-16.5};
        double LXmax{16.5};
        double RXmin{-16.5};
        double RXmax{16.5};
        double RYmin{-12.5};
        double RYmax{12.5};
        double LXmin_abs{1.0};
        double RXmin_abs{1.0};
        double RYmin_abs{0.5};
    };

    struct CfrPhysicalProperties {
        double mass{550.0};
        double width{2.02};
        double depth{1.05};
        double height{1.6};
        double I{1.0 / 12.0 * mass * (width * width + depth * depth)};
        double blade_rad_max{0.5};
        double blade_rad_min{0.3};
        MotorsLimits motors_limits;
    };

    struct EnvParams {
        double mu_1{0.0};   // friction coefficient
        double mu_2{0.2};   // viscocity coefficient
        double mu_3{150.0}; // air drag coefficient
        double mu_4{400.0}; // rotational resistance coefficient
        double g{9.81};
    };

    struct ModelParams {
        double k_left{0.7};    // left rotor damping coefficient
        double k_right{0.7};   // right rotor damping coefficient
        double hy_left{0.02};  // gain of left blade effective radius in y direction
        double hy_right{0.02}; // gain of right blade effective radius in y direction
        double hx_right{0.02}; // gain of right blade effective radius in x direction
    };

    struct ModelInputs {
        double LXMotordeg{0.0};
        double RXMotordeg{0.0};
        double RYMotordeg{0.0};
        double NetTorque{0.0};
        double Motordeglimit{0.0};
        double vx{0.0};
        double vy{0.0};
        double theta_dot{0.0};

        void print(const std::string& node_name) const
        {
            RCLCPP_INFO(rclcpp::get_logger(node_name), "LXMotordeg: %lf", LXMotordeg);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RXMotordeg: %lf", RXMotordeg);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "RYMotordeg: %lf", RYMotordeg);
        }
    };

    struct BladesEffectiveContactRadius {
        double k_left_x{0.0};
        double b_left_x{0.0};
        double effective_rad_LX{0.0};
        double k_right_x{0.0};
        double b_right_x{0.0};
        double effective_rad_RX{0.0};
        double k_right_y{0.0};
        double b_right_y{0.0};
        double effective_rad_RY{0.0};

        void print(const std::string& node_name) const
        {
            RCLCPP_INFO(rclcpp::get_logger(node_name), "k_left_x: %lf", k_left_x);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "b_left_x: %lf", b_left_x);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "effective_rad_LX: %lf",
                        effective_rad_LX);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "k_right_x: %lf", k_right_x);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "b_right_x: %lf", b_right_x);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "effective_rad_RX: %lf",
                        effective_rad_RX);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "k_right_y: %lf", k_right_y);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "b_right_y: %lf", b_right_y);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "effective_rad_RY: %lf",
                        effective_rad_RY);
        };
    };

    struct DynamicsGenerated {
        double Fy_l_rotor{0.0};
        double Fy_r_rotor{0.0};
        double Fy_rotor{0.0};
        double F_friction{0.0};
        double F_drag_y{0.0};
        double Fy{0.0};
        double y_acc{0.0};
        double Fx_r_rotor{0.0};
        double F_drag_x{0.0};
        double Fx{0.0};
        double x_acc{0.0};
        double L_lever{0.0};
        double R_lever{0.0};
        double rot_torque_left{0.0};
        double rot_torque_right{0.0};
        double theta_acc_left{0.0};
        double theta_acc_right{0.0};
        double theta_acc{0.0};

        void print(const std::string& node_name) const
        {
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fy_l_rotor: %lf", Fy_l_rotor);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fy_r_rotor: %lf", Fy_r_rotor);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fy_rotor: %lf", Fy_rotor);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "F_friction: %lf", F_friction);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "F_drag_y: %lf", F_drag_y);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fy: %lf", Fy);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "y_acc: %lf", y_acc);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fx_r_rotor: %lf", Fx_r_rotor);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "F_drag_x: %lf", F_drag_x);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "Fx: %lf", Fx);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "x_acc: %lf", x_acc);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "L_lever: %lf", L_lever);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "R_lever: %lf", R_lever);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "rot_torque_left: %lf", rot_torque_left);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "rot_torque_right: %lf", rot_torque_right);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "theta_acc_left: %lf", theta_acc_left);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "theta_acc_right: %lf", theta_acc_right);
            RCLCPP_INFO(rclcpp::get_logger(node_name), "theta_acc: %lf", theta_acc);
        };
    };

    class CfrDynamics : public rclcpp::Node {
    public:
        CfrDynamics();

    private:
        CfrPhysicalProperties cfr_model_;
        EnvParams environment_params_;
        ModelParams model_params_;
        ModelInputs model_inputs_;
        BladesEffectiveContactRadius effective_c_r_;
        DynamicsGenerated dynamics_gen_;

        rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr actuation_sub_;
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

} // namespace cfr_dynamics_ns

#endif