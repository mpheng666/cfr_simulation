#ifndef CFR_STATE_MACHINE_HPP_
#define CFR_STATE_MACHINE_HPP_

#include "cfr_manager/cfr_manager.hpp"

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

#include <thread>
#include <typeinfo>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace cfr_sm {

    struct EventGetState : sc::event<EventGetState> {
    };
    struct EventGetMode : sc::event<EventGetMode> {
    };
    struct EventSetMode : sc::event<EventSetMode> {
    };
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
        EventControl(const float speed, const float x, const float y, const float z)
            : speed(speed)
            , x(x)
            , y(y)
            , z(z)
        {
        }

        float speed{0.0};
        float x{0.0};
        float y{0.0};
        float z{0.0};
    };

    struct StateIdle;
    struct StateReady;
    struct StateRunning;

    struct IStateRunningManual1;
    struct IStateRunningManual2;
    struct IStateRunningSemiAuto;
    struct IStateRunningMPTAuto;
    struct IStateRunningNYPAuto;

    struct StateStop;
    struct StateError;
    struct StateManual;

    struct sm_CFR : sc::state_machine<sm_CFR, StateIdle> {
    public:
        sm_CFR()
            : sm_CFR_manager_(std::make_shared<cfr_manager::CFRManager>())
        {
            std::cout << "Initiated State Machine: CFR \n";
            std::thread([&]() { rclcpp::spin(sm_CFR_manager_); }).detach();
        }

        std::shared_ptr<cfr_manager::CFRManager> sm_CFR_manager_;

        void unconsumed_event(const sc::event_base& evt)
        {
            std::cout << "Invalid event: " << typeid(evt).name() << "\n";
        };
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
    };

    struct StateReady : sc::state<StateReady, sm_CFR> {
    public:
        StateReady(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateReady \n";
            context<sm_CFR>().sm_CFR_manager_->startBroadcastRobotStatus();
            context<sm_CFR>().sm_CFR_manager_->setAllowMoveBlade(true);
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
            context<sm_CFR>().sm_CFR_manager_->setAllowCmdvel(true);
        }
        ~StateRunning() { std::cout << "Leaving StateRunning \n"; }
        typedef mpl::list<sc::custom_reaction<EventStop>,
                          sc::transition<EventFailed, StateError>,
                          sc::custom_reaction<EventControl>>
        reactions;

        sc::result react(const EventStop& event) { return transit<StateStop>(); }

        sc::result react(const EventControl& event)
        {
            context<sm_CFR>().sm_CFR_manager_->setCmdvel(event.x, event.y, event.z);
            context<sm_CFR>().sm_CFR_manager_->setBladeSpeed(event.speed);
            return discard_event();
        }
    };

    struct StateStop : sc::state<StateStop, sm_CFR> {
    public:
        StateStop(my_context ctx)
            : my_base(ctx)
        {
            std::cout << "State: StateStop \n";
            context<sm_CFR>().sm_CFR_manager_->stopAllActions();
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
            context<sm_CFR>().sm_CFR_manager_->killAllActions();
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