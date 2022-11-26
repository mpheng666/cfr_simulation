#include "cfr_state_machine/cfr_state_machine.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<cfr_sm::sm_CFR> cfr_fsm = std::make_shared<cfr_sm::sm_CFR>();
    rclcpp::Rate loop_rate(10);
    cfr_fsm->initiate();

    while (rclcpp::ok()) {
        char c;
        std::cout << "1=init, 2=start, 3=stop, 4=reset, 5=failed";
        std::cin >> c;
        switch (c) {
            case '1': {
                std::cout << "cfr_sm::EventInit()" << std::endl;
                cfr_fsm->process_event(cfr_sm::EventInit());
                break;
            }
            case '2': {
                std::cout << "cfr_sm::EventStart()" << std::endl;
                cfr_fsm->process_event(cfr_sm::EventStart());
                break;
            }
            case '3': {
                std::cout << "cfr_sm::EventStop()" << std::endl;
                cfr_fsm->process_event(cfr_sm::EventStop());
                break;
            }
            case '4': {
                std::cout << "Tcfr_sm::EventReset()" << std::endl;
                cfr_fsm->process_event(cfr_sm::EventReset());
                break;
            }
            case '5': {
                std::cout << "fr_sm::EventFailed()" << std::endl;
                cfr_fsm->process_event(cfr_sm::EventFailed());
                break;
            }
            case 'q': {
                std::cout << "QUIT!" << std::endl;
                rclcpp::shutdown();
                return 0;
            }
            default: {
                std::cout << "Invalid input" << std::endl;
            }
        };
        rclcpp::spin_some(cfr_fsm);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}