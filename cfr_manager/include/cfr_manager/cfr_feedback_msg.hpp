#ifndef CFR_MANAGER_CFR_FEEDBACK_MSG_HPP_
#define CFR_MANAGER_CFR_FEEDBACK_MSG_HPP_

#include <chrono>
#include <string>

namespace cfr_manager {
    struct CFRFeedbackMsg {
        std::chrono::milliseconds now_ms{
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch())};
        double position_x{0.0};
        double position_y{0.0};
        double theta{0.0};
        double blade_speed{0.0};

        std::string getMsgString() const
        {
            return (std::to_string(now_ms.count()) + ", " + std::to_string(position_x) +
                    ", " + std::to_string(position_y) + ", " + std::to_string(theta) +
                    ", " + std::to_string(blade_speed));
        }
    };
} // namespace cfr_manager

#endif