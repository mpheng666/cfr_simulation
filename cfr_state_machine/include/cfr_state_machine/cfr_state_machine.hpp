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
        sm_CFR(std::shared_ptr<cfr_manager::CFRManager> manager);
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
        typedef mpl::list<sc::transition<EventInitDone, StateReady>,
                          sc::transition<EventFailed, StateError>>
        reactions;
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

    struct StateStopped : sc::state<StateStopped, sm_CFR> {
        StateStopped(my_context ctx);
        typedef mpl::list<sc::transition<EventInit, StateReady>>
        reactions;
    };

    struct StateError : sc::state<StateError, sm_CFR> {
        StateError(my_context ctx);
        typedef sc::custom_reaction<EventInit> reactions;

        sc::result react(const EventInit& event);
    };

} // namespace cfr_sm

#endif