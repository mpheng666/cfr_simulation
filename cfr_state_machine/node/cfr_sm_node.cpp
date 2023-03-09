#include "cfr_state_machine/cfr_state_machine.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto cfr_manager_node = std::make_shared<cfr_manager::CFRManager>();

    cfr_sm::sm_CFR state_machine(cfr_manager_node);

    rclcpp::spin(cfr_manager_node);
    
    state_machine.initiate();
    

    rclcpp::shutdown();
    return 0;
}