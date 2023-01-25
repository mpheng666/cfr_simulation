#include "cfr_state_machine/cfr_state_machine.hpp"

namespace cfr_sm {

    // sm_CFR
    sm_CFR::sm_CFR()
        : sm_CFR_manager_(std::make_shared<cfr_manager::CFRManager>())
    {
        std::thread([&]() { rclcpp::spin(sm_CFR_manager_); }).detach();
    }

    void sm_CFR::unconsumed_event(const sc::event_base& event)
    {
        std::cout << "Invalid event: " << typeid(event).name() << "\n";
    }

    // StateIdle
    StateIdle::StateIdle() { std::cout << "State: StateIdle \n"; }

    sc::result StateIdle::react(const EventInit& event)
    {
        return transit<StateInitializing>();
    }

    // StateInitializing
    StateInitializing::StateInitializing(my_context ctx)
        : my_base(ctx)
    {
        std::cout << "State: StateInitializing \n";
        context<sm_CFR>().sm_CFR_manager_->initialise();
    }

    sc::result StateInitializing::react(const EventInitDone& event)
    {
        return transit<StateReady>();
    }

    // StateReady
    StateReady::StateReady(my_context ctx)
        : my_base(ctx)
    {
        std::cout << "State: StateReady \n";
        context<sm_CFR>().sm_CFR_manager_->startBroadcastRobotStatus();
        context<sm_CFR>().sm_CFR_manager_->setAllowMoveBlade(true);
    }

    sc::result StateReady::react(const EventStart& event)
    {
        return transit<StateRunning>();
    }

    // StateRunning
    StateRunning::StateRunning(my_context ctx)
        : my_base(ctx)
    {
        std::cout << "State: StateRunning \n";
        context<sm_CFR>().sm_CFR_manager_->setAllowCmdvel(true);
    }

    sc::result StateRunning::react(const EventStop& event)
    {
        return transit<StateStop>();
    }

    sc::result StateRunning::react(const EventControl& event)
    {
        context<sm_CFR>().sm_CFR_manager_->setCmdvel(event.x, event.y, event.z);
        context<sm_CFR>().sm_CFR_manager_->setBladeSpeed(event.speed);
        return discard_event();
    }

    // StateStop
    StateStop::StateStop(my_context ctx)
        : my_base(ctx)
    {
        std::cout << "State: StateStop \n";
        context<sm_CFR>().sm_CFR_manager_->stopAllActions();
    }
    // StateError
    StateError::StateError(my_context ctx)
        : my_base(ctx)
    {
        std::cout << "State: StateError \n";
        context<sm_CFR>().sm_CFR_manager_->killAllActions();
    }

    sc::result StateError::react(const EventReset& event) { return transit<StateIdle>(); }

} // namespace cfr_sm