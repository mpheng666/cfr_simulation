#ifndef CFR_PROTOCOL_INTERFACES_PROTOCOL_HPP_
#define CFR_PROTOCOL_INTERFACES_PROTOCOL_HPP_

namespace cfr_protocol {
    enum class Protocol : int {
        GETSTATE,
        GETMODE,
        SETMODE,
        GETFEEDBACK,
        INIT,
        START,
        STOP,
        STARTENGINE,
        CTRL,
        AXIS,
        SETBLADEANGLE,
        BEACONS
    };
}

#endif