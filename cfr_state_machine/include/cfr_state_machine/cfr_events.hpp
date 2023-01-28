#ifndef CFR_SM_EVENTS_HPP_
#define CFR_SM_EVENTS_HPP_

#include "cfr_state_machine/common/boost_sc.hpp"
#include <iostream>

namespace cfr_sm {
    struct EventGetMode : sc::event<EventGetMode> {
    };
    struct EventSetMode : sc::event<EventSetMode> {
    };

    struct EventInit : sc::event<EventInit> {
    };
    struct EventInitDone : sc::event<EventInitDone> {
        EventInitDone() { std::cout << "EventInitDone! \n"; }
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
        EventControl(const float blade_speed, const float linear_x, const float linear_y, const float angular_z)
            : blade_speed(blade_speed)
            , linear_x(linear_x)
            , linear_y(linear_y)
            , angular_z(angular_z)
        {
        }

        float blade_speed{0.0};
        float linear_x{0.0};
        float linear_y{0.0};
        float angular_z{0.0};
    };
} // namespace cfr_sm

#endif