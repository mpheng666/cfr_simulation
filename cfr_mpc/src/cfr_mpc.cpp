#include "cfr_mpc/cfr_mpc.hpp"

namespace cfr_mpc {
    CFRMPC::CFRMPC()
        : Node("cfr_mpc")
        , control_pub_timer_(
          this->create_wall_timer(100ms, std::bind(&CFRMPC::controlPubCb, this)))
        , joy_control_pub_(this->create_publisher<sensor_msgs::msg::Joy>("~/joy_control", 10))
        , cmd_vel_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "~/cmd_vel", 10, std::bind(&CFRMPC::cmdvelCb, this, _1)))
        , odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10, std::bind(&CFRMPC::odomCb, this, _1)))
    {
        r = argInit_struct5_T();
        joy_control_msg_.axes.resize(8);
        joy_control_msg_.buttons.resize(8);
    }

    CFRMPC::~CFRMPC() {}

    void CFRMPC::MPCCompute(const geometry_msgs::msg::Twist& msg)
    {
        struct10_T Info;
        struct4_T statedata;
        argInit_struct4_T(&statedata);

        r.signals.ref[0] = msg.linear.x;
        r.signals.ref[1] = msg.linear.y;
        r.signals.ref[2] = msg.angular.z;

        coder::mpcmoveCodeGeneration(&statedata, &r, u, &Info);
        joy_control_msg_.axes.at(3) = u[0];
        joy_control_msg_.axes.at(2) = u[1];
        joy_control_msg_.axes.at(0) = u[2];

        RCLCPP_INFO_STREAM(this->get_logger(), "plant0: " << statedata.Plant[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "plant1: " << statedata.Plant[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "plant2: " << statedata.Plant[2]);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "disturbance0: " << statedata.Disturbance[0]);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "disturbance1: " << statedata.Disturbance[1]);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "disturbance2: " << statedata.Disturbance[2]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove0: " << statedata.LastMove[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove1: " << statedata.LastMove[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove2: " << statedata.LastMove[2]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ref0: " << r.signals.ref[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ref1: " << r.signals.ref[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ref2: " << r.signals.ref[2]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ym0: " << r.signals.ym[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ym1: " << r.signals.ym[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "signal.ym2: " << r.signals.ym[2]);
    }

    void CFRMPC::cmdvelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        MPCCompute(*msg);
    }

    void CFRMPC::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        r.signals.ym[0] = msg->twist.twist.linear.x;
        r.signals.ym[1] = msg->twist.twist.linear.y;
        r.signals.ym[2] = msg->twist.twist.angular.z;
    }

    void CFRMPC::controlPubCb() { joy_control_pub_->publish(joy_control_msg_); }

    void CFRMPC::argInit_3x1_real_T(double result[3])
    {
        for (int idx0{0}; idx0 < 3; idx0++) {
            result[idx0] = argInit_real_T();
        }
    }

    void CFRMPC::argInit_6x6_real_T(double result[36])
    {
        for (int idx0{0}; idx0 < 6; idx0++) {
            for (int idx1{0}; idx1 < 6; idx1++) {
                result[idx0 + 6 * idx1] = argInit_real_T();
            }
        }
    }

    void CFRMPC::argInit_72x1_boolean_T(boolean_T result[72])
    {
        for (int idx0{0}; idx0 < 72; idx0++) {
            result[idx0] = argInit_boolean_T();
        }
    }

    boolean_T CFRMPC::argInit_boolean_T() { return false; }

    double CFRMPC::argInit_real_T() { return 0.0; }

    void CFRMPC::argInit_struct4_T(struct4_T* result)
    {
        argInit_3x1_real_T(result->Plant);
        argInit_6x6_real_T(result->Covariance);
        argInit_72x1_boolean_T(result->iA);
        result->Disturbance[0] = result->Plant[0];
        result->Disturbance[1] = result->Plant[1];
        result->Disturbance[2] = result->Plant[2];
        result->LastMove[0] = result->Plant[0];
        result->LastMove[1] = result->Plant[1];
        result->LastMove[2] = result->Plant[2];
    }

    struct5_T CFRMPC::argInit_struct5_T()
    {
        struct5_T result;
        result.signals = argInit_struct6_T();
        return result;
    }

    struct6_T CFRMPC::argInit_struct6_T()
    {
        struct6_T result;
        argInit_3x1_real_T(result.ym);
        argInit_3x1_real_T(result.ref);
        return result;
    }

} // namespace cfr_mpc