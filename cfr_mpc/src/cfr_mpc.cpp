#include "cfr_mpc/cfr_mpc.hpp"

namespace cfr_mpc {
    CFRMPC::CFRMPC()
        : Node("cfr_mpc")
        , control_pub_timer_(
          this->create_wall_timer(100ms, std::bind(&CFRMPC::controlPubCb, this)))
        , joy_control_pub_(this->create_publisher<sensor_msgs::msg::Joy>("~/joy", 10))
        , cmd_vel_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
          "~/cmd_vel", 10, std::bind(&CFRMPC::cmdvelCb, this, _1)))
    {
        argInit_struct4_T(&statedata); // set 0
        r = argInit_struct5_T();       // set 0
        joy_control_msg_.axes.resize(8);
        joy_control_msg_.buttons.resize(8);
    }

    CFRMPC::~CFRMPC()
    {
        // mpcmoveCodeGeneration_terminate();
    }

    void CFRMPC::MPCCompute(const geometry_msgs::msg::Twist& msg)
    {
        r.signals.ref[0] = msg.linear.x;
        r.signals.ref[1] = msg.linear.y;
        r.signals.ref[2] = msg.angular.z;
        // statedata = plant[3], disturbance[3], lastmove[3], covariance[36], ia[72]
        // r = signal.ym[3], signal.ref[3]
        // Info = Uopt[33], Yopt[33], Xopt[66], Topt[11], Slack, Iterations, Cost;
        coder::mpcmoveCodeGeneration(&statedata, &r, u, &Info);
        joy_control_msg_.axes.at(1) = u[0];
        joy_control_msg_.axes.at(2) = u[1];
        joy_control_msg_.axes.at(0) = u[2];
        RCLCPP_INFO_STREAM(this->get_logger(), "plant0: " << statedata.Plant[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "plant1: " << statedata.Plant[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "plant2: " << statedata.Plant[2]);
        RCLCPP_INFO_STREAM(this->get_logger(), "disturbance0: " << statedata.Disturbance[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "disturbance1: " << statedata.Disturbance[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "disturbance2: " << statedata.Disturbance[2]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove0: " << statedata.LastMove[0]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove1: " << statedata.LastMove[1]);
        RCLCPP_INFO_STREAM(this->get_logger(), "lastmove2: " << statedata.LastMove[2]);
    }

    void CFRMPC::cmdvelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        MPCCompute(*msg);
    }

    void CFRMPC::controlPubCb() { joy_control_pub_->publish(joy_control_msg_); }

    void CFRMPC::argInit_3x1_real_T(double result[3])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 3; idx0++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0] = argInit_real_T();
        }
    }

    void CFRMPC::argInit_6x6_real_T(double result[36])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 6; idx0++) {
            for (int idx1{0}; idx1 < 6; idx1++) {
                // Set the value of the array element.
                // Change this value to the value that the application requires.
                result[idx0 + 6 * idx1] = argInit_real_T();
            }
        }
    }

    void CFRMPC::argInit_72x1_boolean_T(boolean_T result[72])
    {
        // Loop over the array to initialize each element.
        for (int idx0{0}; idx0 < 72; idx0++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0] = argInit_boolean_T();
        }
    }

    boolean_T CFRMPC::argInit_boolean_T() { return false; }

    double CFRMPC::argInit_real_T() { return 0.0; }

    void CFRMPC::argInit_struct4_T(struct4_T* result)
    {
        argInit_3x1_real_T(result->Plant);      // set all fields to double zero
        argInit_6x6_real_T(result->Covariance); // set all fields to double zero
        argInit_72x1_boolean_T(result->iA);     // set all fields to false
        // set all disturbance and lastmove to plant
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
        // Set the value of each structure field.
        // Change this value to the value that the application requires.
        result.signals = argInit_struct6_T();

        return result;
    }

    struct6_T CFRMPC::argInit_struct6_T()
    {
        struct6_T result;
        // Set the value of each structure field.
        // Change this value to the value that the application requires.
        // result.ref[1]=1;
        // printf("result is %f, %f %f\n",result.ref[0],result.ref[1],result.ref[2]);
        argInit_3x1_real_T(result.ym);
        argInit_3x1_real_T(result.ref);
        return result;
    }

} // namespace cfr_mpc