#include "cfr_manager/cfr_manager.hpp"

namespace cfr_manager {
CFRManager::CFRManager()
    : Node("cfr_manager"),
      feedback_broadcastor_(server_port_, BROADCAST_FREQUENCY_),
      cmd_vel_pub_(
          this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20)),
      blade_speed_pub_(this->create_publisher<std_msgs::msg::Float32>(
          "cfr_blade_speed", 10)),
      odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&CFRManager::odomCallback, this, _1))),
      pub_timer_(this->create_wall_timer(
          100ms, std::bind(&CFRManager::pubTimerCallback, this))),
      update_timer_(this->create_wall_timer(
          50ms, std::bind(&CFRManager::updateTimerCallback, this))) {
  std::cout << "CFR manager is initialised! \n";
}

void CFRManager::startBroadcastRobotStatus() { feedback_broadcastor_.start(); }

void CFRManager::startEngine() {
  std::cout << "Starting engine \n";
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "Engine is started! \n";
}

void CFRManager::initialise() {
  std::cout << "Initializing \n";
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "Initialized! \n";
}

void CFRManager::pubTimerCallback() {
  std_msgs::msg::Float32 blade_speed_msg;
  blade_speed_msg.data = blade_speed_;
  blade_speed_pub_->publish(blade_speed_msg);
  cmd_vel_pub_->publish(twist_curr_);
}

void CFRManager::updateTimerCallback() {
  feedback_msg_.now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  feedback_msg_.position_x = odom_curr_.pose.pose.position.x;
  feedback_msg_.position_y = odom_curr_.pose.pose.position.y;
  feedback_msg_.theta = odom_curr_.pose.pose.orientation.z;
  feedback_msg_.blade_speed = blade_speed_;
  feedback_broadcastor_.updateMsg(feedback_msg_);
}

void CFRManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_curr_ = *msg;
}

void CFRManager::setAllowMoveBlade(const bool command) {
  const std::string command_str = (command) ? "true" : "false";
  std::cout << "Allow move blade: " << command_str << "\n";
  allow_move_blade_ = command;
}

void CFRManager::setAllowCmdvel(const bool command) {
  const std::string command_str = (command) ? "true" : "false";
  std::cout << "Allow command velocity: " << command_str << " \n";
  allow_cmd_vel_ = command;
}

void CFRManager::setBladeSpeed(const float command) {
  if (allow_move_blade_) {
    std::cout << "Blade speed is set at " << command << " \n";
    blade_speed_ = command / 60.0;
  } else {
    std::cout << "Please enable blade! \n";
  }
}

void CFRManager::setCmdvel(const float x, const float y, const float z) {
  if (allow_cmd_vel_) {
    std::cout << "Command velocity linear x is set at " << x << " \n";
    std::cout << "Command velocity linear y is set at " << y << " \n";
    std::cout << "Command velocity angulat z is set at " << z << " \n";
    twist_curr_.linear.x = x;
    twist_curr_.linear.y = y;
    twist_curr_.angular.z = z;
  } else {
    twist_curr_ = geometry_msgs::msg::Twist();
    std::cout << "Please enable engine for command velocity! \n";
  }
}

void CFRManager::stopAllActions() {
  setBladeSpeed(0.0f);
  setCmdvel(0.0f, 0.0f, 0.0f);
}

void CFRManager::killAllActions() {
  stopAllActions();
  setAllowMoveBlade(false);
  setAllowCmdvel(false);
}

} // namespace cfr_manager