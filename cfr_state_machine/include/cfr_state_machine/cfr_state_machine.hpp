#ifndef CFR_STATE_MACHINE_HPP_
#define CFR_STATE_MACHINE_HPP_

#include "cfr_manager/cfr_manager.hpp"

#include "cfr_state_machine/cfr_events.hpp"
#include "cfr_state_machine/cfr_sm_states.hpp"
#include "cfr_state_machine/common/base_reaction.hpp"

#include <thread>
#include <typeinfo>

namespace cfr_sm {
    struct sm_CFR : sc::state_machine<sm_CFR, StateIdle> {
        sm_CFR();
        std::shared_ptr<cfr_manager::CFRManager> sm_CFR_manager_;
        void unconsumed_event(const sc::event_base& event);
    };

    struct StateIdle : sc::simple_state<StateIdle, sm_CFR> {
        StateIdle();
        typedef mpl::list<sc::custom_reaction<EventInit>,
                          sc::transition<EventFailed, StateError>>
        reactions;
        sc::result react(const EventInit& event);
    };

    struct StateInitializing : sc::state<StateInitializing, sm_CFR> {
        StateInitializing(my_context ctx);
        typedef mpl::list<sc::custom_reaction<EventInitDone>,
                          sc::transition<EventFailed, StateError>>
        reactions;

        sc::result react(const EventInitDone& event);
    };

    struct StateReady : sc::state<StateReady, sm_CFR> {
        StateReady(my_context ctx);
        typedef mpl::list<sc::custom_reaction<EventStart>,
                          sc::transition<EventFailed, StateError>>
        reactions;

        sc::result react(const EventStart& event);
    };

    struct StateRunning : sc::state<StateRunning, sm_CFR> {
        StateRunning(my_context ctx);
        typedef mpl::list<sc::custom_reaction<EventStop>,
                          sc::transition<EventFailed, StateError>,
                          sc::custom_reaction<EventControl>>
        reactions;

        sc::result react(const EventStop& event);
        sc::result react(const EventControl& event);
    };

    struct StateStop : sc::state<StateStop, sm_CFR> {
        StateStop(my_context ctx);
        typedef mpl::list<sc::transition<EventInit, StateReady>,
                          sc::transition<EventReset, StateIdle>>
        reactions;
    };

    struct StateError : sc::state<StateError, sm_CFR> {
        StateError(my_context ctx);
        typedef sc::custom_reaction<EventReset> reactions;

        sc::result react(const EventReset& event);
    };

} // namespace cfr_sm

#endif