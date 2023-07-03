#pragma once

#include <memory>
#include <unordered_map>

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "nanotec_driver/mode.hpp"

namespace nanotec_driver
{

class MotorNanotec
{
public:
  MotorNanotec(std::shared_ptr<ros2_canopen::LelyDriverBridge> driver, rclcpp::Logger logger);

  double get_position() {return position_;}
  double get_velocity() {return velocity_;}
  double get_torque() {return torque_;}

  bool set_mode(Mode mode);
  bool set_target(double val);
  Mode get_mode();
  void on_emcy(ros2_canopen::COEmcy emcy);
  bool auto_setup();
  bool switch_off();
  bool switch_enabled();
  bool switch_operational();

  /**
   * @brief Recovers the device from fault
   *
   * This function tries to reset faults and
   * put the device back to operational state.
   *
   */
  bool recover();

  /**
   * @brief Read objects of the drive
   *
   * This function should be called regularly. It reads the status word
   * from the device and translates it into the devices state.
   *
   */
  void read();

  /**
   * @brief Writes objects to the drive
   *
   * This function should be called regularly. It writes the new command
   * word to the drive
   *
   */
  void write();

private:
  bool switch_state(const State402::InternalState & target);
  void register_mode(const std::shared_ptr<ModeHelper> & m);
  std::shared_ptr<ModeHelper> get_mode_helper(Mode mode);

  double position_;
  double velocity_;
  double torque_;

  rclcpp::Logger logger_;

  std::atomic<uint16_t> status_word_;
  uint16_t control_word_;
  std::mutex cw_mutex_;
  std::atomic<bool> start_fault_reset_;
  std::atomic<State402::InternalState> target_state_;

  State402 state_handler_;

  std::unordered_map<Mode, std::shared_ptr<ModeHelper>> modes_;

  Mode selected_mode_;
  std::shared_ptr<ModeHelper> selected_mode_helper_;
  Mode current_mode_;
  std::condition_variable mode_cond_;
  std::mutex mode_mutex_;
  const std::chrono::seconds state_switch_timeout_;

  std::shared_ptr<ros2_canopen::LelyDriverBridge> driver;
};

}  // namespace nanotec_driver
