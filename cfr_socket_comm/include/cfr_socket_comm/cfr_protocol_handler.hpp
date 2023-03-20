#ifndef CFR_SOCKET_COMM_PROTOCOL_HANDLER_HPP_
#define CFR_SOCKET_COMM_PROTOCOL_HANDLER_HPP_

#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <iostream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace cfr_socket_comm {

    using RPY_T = std::array<double, 3>;

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
        int blade_speed_rpm{0};
        double linear_x_relative{0.0};
        double angular_z_relative{0.0};
        double linear_y_relative{0.0};
    };

    using VecStrT = std::vector<std::string>;

    class ProtocolHandler {
    public:
        static std::string
        makeStringTwistSocketFormat(const CFRTwistSocketFormat& input_twist,
                                    const char delimiter)
        {
            std::string retval{"CTRL,"};
            retval.append(std::to_string(input_twist.blade_speed_rpm) + ",");
            retval.append(std::to_string(input_twist.linear_x_relative) + ",");
            retval.append(std::to_string(input_twist.angular_z_relative) + ",");
            retval.append(std::to_string(input_twist.linear_y_relative) + delimiter);
            std::cout << "CONTROL: " << retval << "\n";
            return retval;
        }

        static std::string
        makeStringCFRFeedbackSocketFormat(const CFRFeedbackSocketFormat& input_msg,
                                          const char delimiter)
        {
            std::string retval{};
            retval.append(std::to_string(input_msg.timestamped_ms) + ",");
            retval.append(std::to_string(input_msg.position_x_m) + ",");
            retval.append(std::to_string(input_msg.position_y_m) + ",");
            retval.append(std::to_string(input_msg.theta_deg) + ",");
            retval.append(std::to_string(input_msg.blade_speed_rpm) + ",");
            retval.append(std::to_string(input_msg.blade_angle_deg) + ",");
            retval.append(std::to_string(input_msg.velocity_linear_x) + ",");
            retval.append(std::to_string(input_msg.velocity_linear_y) + ",");
            retval.append(std::to_string(input_msg.velocity_theta) + ",");
            retval.append(std::to_string(input_msg.LX_motor_angle_deg) + ",");
            retval.append(std::to_string(input_msg.RX_motor_angle_deg) + ",");
            retval.append(std::to_string(input_msg.RY_motor_angle_deg));
            retval += delimiter;
            return retval;
        }

        static CFRFeedbackSocketFormat tokenizeOdom(const std::string& odom)
        {
            CFRFeedbackSocketFormat retval;
            const std::string DELIMITER_{","};
            auto splitted_token = splitString(odom, DELIMITER_);
            auto final_token = removeWhiteSpaces(splitted_token);
            if (final_token.size() == 12) {
                retval.timestamped_ms = std::stoul(final_token.at(0));
                retval.position_x_m = std::stod(final_token.at(2));
                retval.position_y_m = -std::stod(final_token.at(1));
                retval.theta_deg = -std::stod(final_token.at(3));
                retval.velocity_linear_x = std::stod(final_token.at(5));
                retval.velocity_linear_y = -std::stod(final_token.at(4));
                retval.velocity_theta = -std::stod(final_token.at(6));
                retval.blade_speed_rpm = std::stod(final_token.at(7));
                retval.blade_angle_deg = std::stod(final_token.at(8));
                retval.LX_motor_angle_deg = std::stod(final_token.at(9));
                retval.RX_motor_angle_deg = std::stod(final_token.at(10));
                retval.RY_motor_angle_deg = std::stod(final_token.at(11));
            }

            return retval;
        }

        static VecStrT tokenizeCommandReply(const std::string& msg)
        {
            const std::string DELIMITER_{","};
            auto splitted_token = splitString(msg, DELIMITER_);
            auto final_token = removeWhiteSpaces(splitted_token);
            return final_token;
        }

        static tf2::Quaternion EulerToQuaternion(const RPY_T& rpy)
        {
            tf2::Quaternion q;
            q.setRPY(rpy.at(0), rpy.at(1), rpy.at(2));
            q.normalize();
            return q;
        }

        static double degToRad(double val) { return (val / 180.0) * M_PI; }

    private:
        static VecStrT splitString(const std::string& input, const std::string& delimiter)
        {
            std::size_t pos_start{0};
            std::size_t pos_end{0};
            std::size_t delim_len{delimiter.length()};
            std::string token;
            VecStrT res{};

            while ((pos_end = input.find(delimiter, pos_start)) != std::string::npos) {
                token = input.substr(pos_start, pos_end - pos_start);
                pos_start = pos_end + delim_len;
                if (token.size()) {
                    res.push_back(token);
                }
            }
            if ((input.substr(pos_start)).size()) {
                res.push_back(input.substr(pos_start));
            }

            return res;
        }

        static VecStrT removeWhiteSpaces(VecStrT& input)
        {
            for (auto& token : input) {
                token.erase(std::remove_if(token.begin(), token.end(),
                                           [](char c) {
                                               return (c == ' ' || c == '\n' ||
                                                       c == '\r' || c == '\t' ||
                                                       c == '\v' || c == '\f');
                                           }),
                            token.end());
            }
            return input;
        }

        VecStrT init_protocol_;
    };
} // namespace cfr_socket_comm

#endif
