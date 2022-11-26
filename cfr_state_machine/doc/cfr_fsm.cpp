#include <iostream>

namespace cfr_fsm {

    enum class Event { INIT = 0, START, STOP, RESET, NOTHING };
    enum class State { IDLE = 0, READY, RUNNING, STOPPED, ERRORED };

    class CFRFSM {

    public:
        void start()
        {
            run_fsm(current_event_);
        }

        void setEvent(const Event& e) { current_event_ = e; }
        State getState() const { return current_state_; }

    private:
        State current_state_{State::IDLE};
        Event current_event_{Event ::NOTHING};

        void run_fsm(const Event& e)
        {
            switch (current_state_) {
                case State::IDLE: {
                    if (current_event_ == Event::INIT) {
                        current_state_ = State::READY;
                    }
                    std::cout << "Current State: "
                              << "IDLE \n";
                    current_event_ = Event::NOTHING;
                    break;
                }
                case State::READY: {
                    if (current_event_ == Event::START) {
                        current_state_ = State::RUNNING;
                    }
                    else if (current_event_ == Event::STOP) {
                        current_state_ = State::STOPPED;
                    }
                    std::cout << "Current State: "
                              << "READY \n";
                    current_event_ = Event::NOTHING;
                    break;
                }
                case State::RUNNING: {
                    if (current_event_ == Event::STOP) {
                        current_state_ = State::STOPPED;
                    }
                    std::cout << "Current State: "
                              << "RUNNING \n";
                    current_event_ = Event::NOTHING;
                    break;
                }
                case State::STOPPED: {
                    if (current_event_ == Event::INIT) {
                        current_state_ = State::READY;
                    }
                    else if (current_event_ == Event::RESET) {
                        current_state_ = State::IDLE;
                    }
                    std::cout << "Current State: "
                              << "STOPPED \n";
                    current_event_ = Event::NOTHING;
                    break;
                }
                case State::ERRORED: {
                    if (current_event_ == Event::INIT) {
                        current_state_ = State::READY;
                    }
                    else if (current_event_ == Event::RESET) {
                        current_state_ = State::IDLE;
                    }
                    std::cout << "Current State: "
                              << "ERRORED \n";
                    current_event_ = Event::NOTHING;
                    break;
                }
                default:
                {
                    std::cout << "NOTHING \n";
                    break;
                }
            }
        }
    };

} // namespace cfr_fsm
