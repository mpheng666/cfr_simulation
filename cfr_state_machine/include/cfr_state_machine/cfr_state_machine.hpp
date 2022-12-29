#ifndef CFR_STATE_MACHINE_HPP_
#define CFR_STATE_MACHINE_HPP_

#include "cfr_manager/cfr_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
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
    struct EventStatus : sc::event<EventStatus> {
    };
    struct EventFeedBack : sc::event<EventFeedBack> {
    };
    struct EventControl : sc::event<EventControl> {
    };

    struct StateIdle;
    struct StateReady;
    struct StateRunning;
    struct StateStop;
    struct StateError;
    struct StateManual;

    struct sm_CFR : sc::state_machine<sm_CFR, StateIdle> {
    public:
        sm_CFR() { std::cout << "Initiated State Machine: CFR \n"; }
        cfr_manager::CFRManager sm_CFR_manager_;
    };

    struct StateIdle : sc::simple_state<StateIdle, sm_CFR> {
    public:
        StateIdle() { std::cout << "State: StateIdle \n"; }
        ~StateIdle() { std::cout << "Leaving StateIdle \n"; }
        typedef mpl::list<sc::custom_reaction<EventInit>,
                          sc::transition<EventFailed, StateError>>
        reactions;

        sc::result react(const EventInit& event)
        {
            // std::cout << "Received EventInit at StateIdle \n";
            return transit<StateReady>();
        }

        void unconsumed_event(const sc::event_base& e){
            std::cout << "unconsumed event \n";
        };
    };

    struct StateReady : sc::state<StateReady, sm_CFR> {
    public:
        StateReady(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateReady \n";
            context<sm_CFR>().sm_CFR_manager_.startBroadcastRobotStatus(true);
            context<sm_CFR>().sm_CFR_manager_.setAllowMoveBlade(true);
        }
        ~StateReady() { std::cout << "Leaving StateReady \n"; }
        typedef mpl::list<sc::custom_reaction<EventStart>,
                          sc::transition<EventFailed, StateError>>
        reactions;

        sc::result react(const EventStart& event)
        {
            // std::cout << "Received EventStart at StateReady \n";
            return transit<StateRunning>();
        }
    };

    struct StateRunning : sc::state<StateRunning, sm_CFR> {
    public:
        StateRunning(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateRunning \n";
            context<sm_CFR>().sm_CFR_manager_.setAllowCmdvel(true);
        }
        ~StateRunning() { std::cout << "Leaving StateRunning \n"; }
        typedef mpl::list<sc::custom_reaction<EventStop>,
                          sc::transition<EventFailed, StateError>>
        reactions;

        sc::result react(const EventStop& event)
        {
            // std::cout << "Received EventStop at StateRunning \n";
            return transit<StateStop>();
        }
    };

    struct StateStop : sc::state<StateStop, sm_CFR> {
    public:
        StateStop(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateStop \n";
            context<sm_CFR>().sm_CFR_manager_.stopAllActions();
        }
        ~StateStop() { std::cout << "Leaving StateStop \n"; }
        typedef mpl::list<sc::transition<EventInit, StateReady>,
                          sc::transition<EventReset, StateIdle>>
        reactions;
    };

    struct StateError : sc::state<StateError, sm_CFR> {
    public:
        StateError(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateError \n";
            context<sm_CFR>().sm_CFR_manager_.killAllActions();
        }
        ~StateError() { std::cout << "Leaving StateError \n"; }
        typedef sc::custom_reaction<EventReset> reactions;

        sc::result react(const EventReset& event) { return transit<StateIdle>(); }
    };

    struct StateManual : sc::simple_state<StateManual, sm_CFR> {
    public:
        StateManual() { std::cout << "State: StateManual \n"; }
    };

} // namespace cfr_sm

#endif