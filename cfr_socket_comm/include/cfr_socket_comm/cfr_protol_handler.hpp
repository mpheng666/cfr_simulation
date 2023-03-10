#ifndef CFR_SOCKET_COMM_PROTOCOL_HANDLER_HPP_
#define CFR_SOCKET_COMM_PROTOCOL_HANDLER_HPP_

#include <string>
#include <vector>

namespace cfr_socket_comm {
    struct CFRFeedbackSocketFormat {
        uint32_t timestamped_ms{0};
        double position_x_m{0.0};
        double position_y_m{0.0};
        double theta_deg{0.0};
        double blade_speed_rpm{0.0};
        double blade_angle_deg{0.0};
        double velocity_linear_x{0.0};
        double velocity_linear_y{0.0};
        double velocity_theta{0.0};
        double LX_motor_angle_deg{0.0};
        double RX_motor_angle_deg{0.0};
        double RY_motor_angle_deg{0.0};
    };

    struct CFRTwistSocketFormat {
        double blade_speed_rpm{0.0};
        double linear_x_relative{0.0};
        double angular_z_relative{0.0};
        double linear_y_relative{0.0};
    };

    using VecStrT = std::vector<std::string>;

    class ProtocolHandler {
    public:
        static std::string makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist, const char delimiter)
        {
            std::string retval{"CTRL,"};
            retval.append(std::to_string(input_twist.blade_speed_rpm) + ",");
            retval.append(std::to_string(input_twist.linear_x_relative) + ",");
            retval.append(std::to_string(input_twist.angular_z_relative) + ",");
            retval.append(std::to_string(input_twist.linear_y_relative) + delimiter);
            return retval;
        }

        void tokenizeOdom()
        {

        }

        VecStrT init_protocol_;

        
    };
} // namespace cfr_socket_comm

#endif