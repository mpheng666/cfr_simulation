#ifndef CFR_SM_STATES_HPP_
#define CFR_SM_STATES_HPP_

#include "cfr_state_machine/common/boost_sc.hpp"

namespace cfr_sm
{
    struct StateIdle;
    struct StateInitializing;
    struct StateReady;
    struct StateRunning;

    struct IStateRunningManual1;
    struct IStateRunningManual2;
    struct IStateRunningSemiAuto;
    struct IStateRunningMPTAuto;
    struct IStateRunningNYPAuto;

    struct StateStopped;
    struct StateError;
}

#endif