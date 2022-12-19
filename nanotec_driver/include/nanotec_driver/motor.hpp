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

  bool setTarget(double val);
  bool enterModeAndWait(int8_t mode);
  bool isModeSupported(int8_t mode);
  int8_t getMode();
  bool readState();

  bool autoSetup();

  /**
   * @brief Initialise the drive
   */
  bool init();

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

  /**
   * @brief Shutdowns the drive
   *
   * This function shuts down the drive by bringing it into
   * SwitchOn disabled state.
   *
   */
  bool shutdown();

  /**
   * @brief Executes a quickstop
   *
   * The function executes a quickstop.
   *
   */
  bool halt();

  /**
   * @brief Recovers the device from fault
   *
   * This function tries to reset faults and
   * put the device back to operational state.
   *
   */
  bool recover();

  /**
   * @brief Register a new operation mode for the drive
   *
   * This function will register an operation mode for the drive.
   * It will check if the mode is supported by the drive by reading
   * 0x6508 object.
   *
   * @tparam T
   * @tparam Args
   * @param mode
   * @param args
   * @return true
   * @return false
   */
  template<typename T, typename ... Args>
  bool registerMode(int8_t mode, Args &&... args)
  {
    return mode_allocators_.insert(
      std::make_pair(
        mode, [args ..., mode, this]()
        {
          if (isModeSupportedByDevice(mode)) {
            registerMode(mode, ModeSharedPtr(new T(args ...)));
          }
        }))
           .second;
  }

private:
  virtual bool isModeSupportedByDevice(int8_t mode);
  void registerMode(int8_t id, const ModeSharedPtr & m);

  ModeSharedPtr allocMode(int8_t mode);

  bool switchMode(int8_t mode);
  bool switchState(const State402::InternalState & target);

  rclcpp::Logger logger_;

  std::atomic<uint16_t> status_word_;
  uint16_t control_word_;
  std::mutex cw_mutex_;
  std::atomic<bool> start_fault_reset_;
  std::atomic<State402::InternalState> target_state_;

  State402 state_handler_;

  std::mutex map_mutex_;
  std::unordered_map<int8_t, ModeSharedPtr> modes_;
  typedef std::function<void ()> AllocFuncType;
  std::unordered_map<int8_t, AllocFuncType> mode_allocators_;

  ModeSharedPtr selected_mode_;
  int8_t mode_id_;
  std::condition_variable mode_cond_;
  std::mutex mode_mutex_;
  const std::chrono::seconds state_switch_timeout_;

  std::shared_ptr<LelyMotionControllerBridge> driver;
  std::shared_ptr<RemoteObject> status_word_entry_;
  std::shared_ptr<RemoteObject> control_word_entry_;
  std::shared_ptr<RemoteObject> op_mode_display_;
  std::shared_ptr<RemoteObject> op_mode_;
  std::shared_ptr<RemoteObject> supported_drive_modes_;
  const uint16_t status_word_entry_index = 0x6041;
  const uint16_t control_word_entry_index = 0x6040;
  const uint16_t op_mode_display_index = 0x6061;
  const uint16_t op_mode_index = 0x6060;
  const uint16_t supported_drive_modes_index = 0x6502;
};

}  // namespace nanotec_driver
