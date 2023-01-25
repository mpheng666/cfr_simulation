#ifndef CFR_SM_EVENTS_HPP_
#define CFR_SM_EVENTS_HPP_

#include "cfr_state_machine/common/boost_sc.hpp"

namespace cfr_sm {
    struct EventGetState : sc::event<EventGetState> {
    };
    struct EventGetMode : sc::event<EventGetMode> {
    };
    struct EventSetMode : sc::event<EventSetMode> {
    };

    struct EventInit : sc::event<EventInit> {
    };
    struct EventInitDone : sc::event<EventInitDone> {
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
} // namespace cfr_sm

#endif