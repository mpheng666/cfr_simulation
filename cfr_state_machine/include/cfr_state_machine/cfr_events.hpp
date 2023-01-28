#ifndef CFR_SM_EVENTS_HPP_
#define CFR_SM_EVENTS_HPP_

#include "cfr_state_machine/common/boost_sc.hpp"
#include <iostream>

namespace cfr_sm {
    struct EventGetMode : sc::event<EventGetMode> {
    };
    struct EventSetMode : sc::event<EventSetMode> {
        EventSetMode(int mode)
            : mode(mode)
        {
        }
        int mode = 0;
    };
    struct EventFeedBack : sc::event<EventFeedBack> {
        EventFeedBack(bool is_run)
            : is_run(is_run)
        {
        }
        bool is_run{false};
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
    struct EventStartEngine : sc::event<EventStartEngine> {
    };

    struct EventFailed : sc::event<EventFailed> {
    };

    struct EventControl : sc::event<EventControl> {
        EventControl(const float blade_speed,
                     const float linear_x,
                     const float linear_y,
                     const float angular_z)
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

    struct EventAxis : sc::event<EventAxis> {
        EventAxis(const float blade_speed, const float LX, const float RX, const float RY)
            : blade_speed(blade_speed)
            , LX(LX)
            , RX(RX)
            , RY(RY)
        {
        }

        float blade_speed{0.0};
        float LX{0.0};
        float RX{0.0};
        float RY{0.0};
    };

    struct EventSetBladeAngle : sc::event<EventSetBladeAngle> {
        EventSetBladeAngle(float blade_speed)
            : blade_speed(blade_speed)
        {
        }
        float blade_speed{0.0};
    };

    struct EventBeacons : sc::event<EventBeacons> {
    };

} // namespace cfr_sm

#endif