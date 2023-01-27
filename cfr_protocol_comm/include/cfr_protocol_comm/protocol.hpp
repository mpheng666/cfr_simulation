#ifndef CFR_PROTOCOL_COMM_PROTOCOL_HPP_
#define CFR_PROTOCOL_COMM_PROTOCOL_HPP_

#include <string>
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

    class ProtocolConversion {
    public:
        static std::string getStringFromProtocol(const Protocol& protocol)
        {
            switch (protocol) {
                case Protocol::GETSTATE:
                    return "PSTATE";
                    break;
                case Protocol::GETMODE:
                    return "MODE";
                    break;
                case Protocol::SETMODE:
                    return "NYPAUTO";
                    break;
                case Protocol::GETFEEDBACK:
                    return "FB";
                    break;
                case Protocol::INIT:
                    return "INIT";
                    break;
                case Protocol::START:
                    return "START";
                    break;
                case Protocol::STOP:
                    return "STOP";
                    break;
                case Protocol::STARTENGINE:
                    return "STARTENGINE";
                    break;
                case Protocol::CTRL:
                    return "CTRL";
                    break;
                case Protocol::AXIS:
                    return "AXIS";
                    break;
                case Protocol::SETBLADEANGLE:
                    return "BLADEANG";
                    break;
                case Protocol::BEACONS:
                    return "BEACONS";
                    break;
            }
        }

        Protocol getProtocolFromString(const std::string& data) const
        {
            if (data == "PSTATE")
                return Protocol::GETSTATE;
            if (data == "MODE")
                return Protocol::GETMODE;
            if (data == "NYPAUTO")
                return Protocol::SETMODE;
            if (data == "FB")
                return Protocol::GETFEEDBACK;
            if (data == "INIT")
                return Protocol::INIT;
            if (data == "START")
                return Protocol::START;
            if (data == "STOP")
                return Protocol::STOP;
            if (data == "STARTENGINE")
                return Protocol::STARTENGINE;
            if (data == "CTRL")
                return Protocol::CTRL;
            if (data == "AXIS")
                return Protocol::AXIS;
            if (data == "BLADEANG")
                return Protocol::SETBLADEANGLE;
            if (data == "BEACONS")
                return Protocol::BEACONS;
            return {};
        }
    };
} // namespace cfr_protocol

#endif