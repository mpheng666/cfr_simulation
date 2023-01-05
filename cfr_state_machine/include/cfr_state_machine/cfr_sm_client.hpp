#ifndef CFR_SM_CLIENT_HPP_
#define CFR_SM_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cfr_sm_client {
    enum class CFRSMServiceType { STATUS, FB, INIT, START, STOP, RESET, /* CTRL, */ UNKNOWN };

    class CFRSMClient : public rclcpp::Node {
    public:
        CFRSMClient(const std::string& client_name);
        void callCFRService(const std::string& command);
        void callControl(const std::vector<float>& command);

    private:
        const std::string CLIENT_NS_{"cfr_sm_node/"};
        std::string client_name_{};

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr control_pub_;

        static CFRSMServiceType strCmdToService(const std::string& command)
        {
            if (command == "STATUS")
                return CFRSMServiceType::STATUS;
            if (command == "FB")
                return CFRSMServiceType::FB;
            if (command == "INIT")
                return CFRSMServiceType::INIT;
            if (command == "START")
                return CFRSMServiceType::START;
            if (command == "STOP")
                return CFRSMServiceType::STOP;
            if (command == "RESET")
                return CFRSMServiceType::RESET;
            return CFRSMServiceType::UNKNOWN;
        }

        static constexpr const char* serviceToName(CFRSMServiceType service_type) noexcept
        {
            switch (service_type) {
                case CFRSMServiceType::STATUS:
                    return "status_service";
                case CFRSMServiceType::FB:
                    return "feedback_service";
                case CFRSMServiceType::INIT:
                    return "init_service";
                case CFRSMServiceType::START:
                    return "start_service";
                case CFRSMServiceType::STOP:
                    return "stop_service";
                case CFRSMServiceType::RESET:
                    return "reset_service";
                case CFRSMServiceType::UNKNOWN:
                    return "unknown_service";
            }
            return "invalid_service";
        }
        
    };
} // namespace cfr_sm_client

#endif