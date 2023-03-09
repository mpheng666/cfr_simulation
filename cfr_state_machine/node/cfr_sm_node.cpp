#include "cfr_state_machine/cfr_state_machine.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    cfr_sm::sm_CFR state_machine;
    state_machine.initiate();
    
    // rclcpp::shutdown();
    return 0;
}