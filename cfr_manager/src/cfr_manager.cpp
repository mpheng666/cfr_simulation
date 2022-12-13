#include "cfr_manager/cfr_manager.hpp"

namespace cfr_manager {
    CFRManager::CFRManager() {}

    void CFRManager::startBroadcastRobotStatus(const bool command)
    {
        std::cout << "Start channel 10001 to feedback current status \n";
    }

    void CFRManager::setAllowMoveBlade(const bool command)
    {
        const std::string command_str = (command) ? "true" : "false";
        std::cout << "Allow move blade: " << command_str << "\n";
        allow_move_blade_ = command;
    }

    void CFRManager::setAllowCmdvel(const bool command)
    {
        const std::string command_str = (command) ? "true" : "false";
        std::cout << "Allow command velocity: " << command_str << " \n";
        allow_cmd_vel_ = command;
    }

    void CFRManager::setBladeSpeed(const float command)
    {
        if (allow_move_blade_) {
            std::cout << "Blade speed is set at " << command << " \n";
        }
        else {
            std::cout << "Please enable blade! \n";
        }
    }

    void CFRManager::setCmdvel(const float x, const float y, const float z)
    {
        if (allow_cmd_vel_) {

            std::cout << "Command velocity linear x is set at " << x << " \n";
            std::cout << "Command velocity linear y is set at " << y << " \n";
            std::cout << "Command velocity angulat z is set at " << z << " \n";
        }
        else {
            std::cout << "Please enable engine for command velocity! \n";
        }
    }

    void CFRManager::stopAllActions()
    {
        setBladeSpeed(0.0f);
        setCmdvel(0.0f, 0.0f, 0.0f);
    }

    void CFRManager::killAllActions()
    {
        stopAllActions();
        setAllowMoveBlade(false);
        setAllowCmdvel(false);
    }

} // namespace cfr_manager