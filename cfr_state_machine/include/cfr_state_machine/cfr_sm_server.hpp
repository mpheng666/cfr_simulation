#ifndef CFR_SM_SERVER_HPP_
#define CFR_SM_SERVER_HPP_

#include "cfr_state_machine/cfr_state_machine.hpp"
#include "cfr_state_machine/srv/start.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>

namespace cfr_sm {
    class CFRControlServer : public rclcpp::Node {
    public:
        CFRControlServer();
        void start();

    private:
        sm_CFR cfr_sm_;
        
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    };
} // namespace cfr_sm

#endif