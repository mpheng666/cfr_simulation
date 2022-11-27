#include "cfr_state_machine/cfr_sm_server.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<cfr_sm::CFRControlServer> cfr_sm_server =
    std::make_shared<cfr_sm::CFRControlServer>();
    cfr_sm_server->start();
    
    rclcpp::spin(cfr_sm_server);
    rclcpp::shutdown();
    return 0;
}