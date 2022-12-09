#ifndef CFR_SM_CLIENT_HPP_
#define CFR_SM_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cfr_sm_client {
    enum class CFRSMServiceType { INIT, START, STOP, RESET, UNKNOWN };

    class CFRSMClient {
    public:
        CFRSMClient(const std::string& client_name);
        void callCFRService(const std::string& command);

    private:
        CFRSMServiceType strCmdToService(const std::string& command)
        {
            if (command == "INIT")
                return CFRSMServiceType::INIT;
            else if (command == "START")
                return CFRSMServiceType::START;
            else if (command == "STOP")
                return CFRSMServiceType::STOP;
            else if (command == "RESET")
                return CFRSMServiceType::RESET;
            else
                return CFRSMServiceType::UNKNOWN;
        }

        constexpr const char* serviceToName(CFRSMServiceType service_type) noexcept
        {
            switch (service_type) {
                case CFRSMServiceType::INIT:
                    return "init_service";
                case CFRSMServiceType::STOP:
                    return "stop_service";
                case CFRSMServiceType::START:
                    return "start_service";
                case CFRSMServiceType::RESET:
                    return "reset_service";
                case CFRSMServiceType::UNKNOWN:
                    return "unknown_service";
            }
            return "invalid_service";
        }
        
        const std::string CLIENT_NS_{"cfr_sm_node/"};
        std::string client_name_{};
    };
} // namespace cfr_sm_client

#endif