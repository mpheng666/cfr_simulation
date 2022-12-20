#include "cfr_manager/cfr_manager.hpp"

namespace cfr_manager {
    CFRManager::CFRManager()
        : Node("cfr_manager")
        , cmd_vel_pub_(
          this->create_publisher<geometry_msgs::msg::Twist>("cfr_cmd_vel", 20))
        , blade_speed_pub_(
          this->create_publisher<std_msgs::msg::Float32>("cfr_blade_speed", 10))
        , odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&CFRManager::odomCallback, this, _1)))
        , pub_timer_(
          this->create_wall_timer(10ms, std::bind(&CFRManager::pubTimerCallback, this)))
    {
    }

    void CFRManager::startBroadcastRobotStatus(const bool command)
    {
        std::cout << "Start channel 10001 to feedback current status \n";
    }

    void CFRManager::pubTimerCallback() {}

    void CFRManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {}

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