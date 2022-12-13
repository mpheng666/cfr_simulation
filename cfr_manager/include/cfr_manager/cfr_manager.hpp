#ifndef CFR_SM_CFR_MANAGER_HPP_
#define CFR_SM_CFR_MANAGER_HPP_

#include <iostream>
#include <string>

namespace cfr_manager {
    class CFRManager {
    public:
        CFRManager();
        void startBroadcastRobotStatus(const bool command);
        void setAllowMoveBlade(const bool command);
        void setAllowCmdvel(const bool command);
        void setBladeSpeed(const float command);
        void setCmdvel(const float x, const float y, const float z);
        void stopAllActions();
        void killAllActions();

    private:
        bool allow_move_blade_{false};
        bool allow_cmd_vel_{false};
        float blade_speed_{0.0f};
        
    };
} // namespace cfr_manager

#endif