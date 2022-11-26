#include "src/cfr_fsm.cpp"
#include <chrono>
#include <future>
#include <thread>

static char getInput()
{
    char input;
    std::cout << "Please input state: \n";
    std::cin >> input;
    return input;
}

int main(int argc, char** argv)
{
    cfr_fsm::CFRFSM cfr_state_machine;
    cfr_state_machine.start();

    bool done{false};

    while (!done) {

        std::chrono::seconds timeout(5);
        std::future<char> future = std::async(getInput);
        char input = '.';
        if (future.wait_for(timeout) == std::future_status::ready)
            input = future.get();

        switch (input) {
            case 'i':
            {
                cfr_state_machine.setEvent(cfr_fsm::Event::INIT);
                std::cout << "Input: " << input << '\n';
                break;
            }
            case 's':
            {
                cfr_state_machine.setEvent(cfr_fsm::Event::START);
                std::cout << "Input: " << input << '\n';
                break;
            }
            case 't':
            {
                cfr_state_machine.setEvent(cfr_fsm::Event::STOP);
                std::cout << "Input: " << input << '\n';
                break;
            }
            case 'r':
            {
                cfr_state_machine.setEvent(cfr_fsm::Event::RESET);
                std::cout << "Input: " << input << '\n';
                break;
            }
            case 'n':
            {
                cfr_state_machine.setEvent(cfr_fsm::Event::NOTHING);
                std::cout << "Input: " << input << '\n';
                break;
            }
            case 'x': {
                std::cout << "Input: " << input << '\n';
                break;
            }
            default:
                break;
        }
        cfr_state_machine.start();
    }

    std::cout << "DONE!";

    return 0;
}
