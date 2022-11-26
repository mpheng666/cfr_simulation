#ifndef CFR_STATE_MACHINE_HPP_
#define CFR_STATE_MACHINE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

#include <iostream>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace cfr_sm {

    struct EventInit : sc::event<EventInit> {
    };
    struct EventStart : sc::event<EventStart> {
    };
    struct EventStop : sc::event<EventStop> {
    };
    struct EventReset : sc::event<EventReset> {
    };
    struct EventFailed : sc::event<EventFailed> {
    };

    struct StateIdle;
    struct StateReady;
    struct StateRunning;
    struct StateStop;
    struct StateError;
    struct StateManual;
    

    struct sm_CFR : sc::state_machine<sm_CFR, StateIdle>, public rclcpp::Node {
    public:
        sm_CFR()
            : Node("cfr_sm_node")
        {
            std::cout << "Initiated State Machine: CFR \n";
        }
    };

    struct StateIdle : sc::simple_state<StateIdle, sm_CFR> {
    public:
        StateIdle() { std::cout << "State: StateIdle \n"; }
        typedef mpl::list<sc::transition<EventInit, StateReady>,
                          sc::transition<EventFailed, StateError>>
        reactions;
    };

    struct StateReady : sc::simple_state<StateReady, sm_CFR> {
    public:
        StateReady() { std::cout << "State: StateReady \n"; }
        typedef mpl::list<sc::transition<EventStart, StateRunning>,
                          sc::transition<EventFailed, StateError>>
        reactions;
    };

    struct StateRunning : sc::simple_state<StateRunning, sm_CFR> {
    public:
        StateRunning() { std::cout << "State: StateRunning \n"; }
        typedef mpl::list<sc::transition<EventStop, StateStop>,
                          sc::transition<EventFailed, StateIdle>,
                          sc::transition<EventFailed, StateError>>
        reactions;
    };

    struct StateStop : sc::simple_state<StateStop, sm_CFR> {
    public:
        StateStop() { std::cout << "State: StateStop \n"; }
        typedef mpl::list<sc::transition<EventInit, StateReady>,
                          sc::transition<EventReset, StateIdle>>
        reactions;
    };

    struct StateError : sc::simple_state<StateError, sm_CFR> {
    public:
        StateError() { std::cout << "State: StateError \n"; }
        typedef sc::transition<EventReset, StateIdle> reactions;
    };

    // struct StateManual : sc::simple_state<StateManual, sm_CFR> {
    // public:
    //     StateManual() { std::cout << "State: StateManual \n"; }
    //     typedef mpl::list<sc::transition<event_MoveToSecondState, secondState>,
    //                       sc::transition<event_MoveToThirdState, thirdState>>
    //     reactions;
    // };

} // namespace cfr_sm

#endif