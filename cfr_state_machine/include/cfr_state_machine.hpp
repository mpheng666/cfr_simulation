#ifndef CFR_STATE_MACHINE_HPP_
#define CFR_STATE_MACHINE_HPP_

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>

#include <iostream>

namespace sc = boost::statechart;

namespace cfr_sm {

    class EventInit : sc::event<EventInit> {
    };
    class EventStart : sc::event<EventStart> {
    };
    class EventStop : sc::event<EventStop> {
    };
    class EventReset : sc::event<EventReset> {
    };

    class StateIdle;
    class StateReady;
    class StateRunning;
    class StateStop;
    class StateError;
    class StateManual;

    class sm_CFR : sc::state_machine<sm_CFR, StateIdle> {
        sm_CFR() { std::cout << "State Machine: CFR \n"; }
    };

    class StateIdle : sc::simple_state<StateIdle, sm_CFR> {
        StateIdle() { std::cout << "State: StateIdle \n"; }
    };

    class StateReady : sc::simple_state<StateReady, sm_CFR> {
        StateReady() { std::cout << "State: StateReady \n"; }
    };
    class StateRunning : sc::simple_state<StateRunning, sm_CFR> {
        StateRunning() { std::cout << "State: StateRunning \n"; }
    };
    class StateStop : sc::simple_state<StateStop, sm_CFR> {
        StateStop() { std::cout << "State: StateStop \n"; }
    };
    class StateError : sc::simple_state<StateError, sm_CFR> {
        StateError() { std::cout << "State: StateError \n"; }
    };
    class StateManual : sc::simple_state<StateManual, sm_CFR> {
        StateManual() { std::cout << "State: StateManual \n"; }
    };

} // namespace cfr_sm

#endif