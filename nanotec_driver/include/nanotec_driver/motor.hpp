#pragma once

#include <memory>
#include <unordered_map>

#include "canopen_402_driver/motor.hpp"

#include "nanotec_driver/auto_setup.hpp"

using ros2_canopen::State402;
using ros2_canopen::Command402;
using ros2_canopen::MotorBase;

namespace nanotec_driver
{

class MotorNanotec
{
public:
  enum OperationMode
  {
    Auto_Setup = -2,
  };

  MotorNanotec(std::shared_ptr<LelyMotionControllerBridge> driver, rclcpp::Logger logger);

  double get_position() {return position_;}
  double get_velocity() {return velocity_;}
  double get_torque() {return torque_;}

  bool set_mode(int8_t mode);
  bool set_target(double val);
  int8_t get_mode_id();
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
  void register_mode(const ModeSharedPtr & m);
  ModeSharedPtr get_mode(int8_t mode_id);

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

  std::unordered_map<int8_t, ModeSharedPtr> modes_;

  ModeSharedPtr selected_mode_;
  int8_t selected_mode_id_;
  int8_t current_mode_id_;
  std::condition_variable mode_cond_;
  std::mutex mode_mutex_;
  const std::chrono::seconds state_switch_timeout_;

  std::shared_ptr<LelyMotionControllerBridge> driver;
  std::shared_ptr<RemoteObject> status_word_entry_;
  std::shared_ptr<RemoteObject> control_word_entry_;
  std::shared_ptr<RemoteObject> op_mode_display_;
  std::shared_ptr<RemoteObject> op_mode_;
  std::shared_ptr<RemoteObject> position_actual_value_;
  std::shared_ptr<RemoteObject> velocity_actual_value_;
  std::shared_ptr<RemoteObject> torque_actual_value_;

  const uint16_t status_word_entry_index = 0x6041;
  const uint16_t control_word_entry_index = 0x6040;
  const uint16_t op_mode_display_index = 0x6061;
  const uint16_t op_mode_index = 0x6060;
  const uint16_t position_actual_value_index = 0x6064;
  const uint16_t velocity_actual_value_index = 0x606C;
  const uint16_t torque_actual_value_index = 0x6077;
};

}  // namespace nanotec_driver
