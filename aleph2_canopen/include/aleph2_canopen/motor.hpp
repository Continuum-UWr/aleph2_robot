#pragma once

#include "canopen_402_driver/motor.hpp"

using ros2_canopen::State402;
using ros2_canopen::Command402;

namespace aleph2_canopen
{

class AutoSetupMode : public ros2_canopen::Mode
{
protected:
  std::shared_ptr<LelyMotionControllerBridge> driver;
  std::atomic_bool execute_;

  enum SW_bits
  {
    SW_AutoSetupCompleted = State402::SW_Operation_mode_specific0,
  };

  enum CW_bits
  {
    CW_StartAutoSetup = Command402::CW_Operation_mode_specific0,
  };

public:
  AutoSetupMode(std::shared_ptr<LelyMotionControllerBridge> driver);

  virtual bool start() override;
  virtual bool read(const uint16_t &sw) override;
  virtual bool write(OpModeAccesser &cw) override;
  virtual bool executeAutoSetup();
};

class MotorNanotec : public MotorBase
{
public:
  enum OperationMode {
    Auto_Setup = -2,
  };

  MotorNanotec(std::shared_ptr<LelyMotionControllerBridge> driver);

  virtual bool setTarget(double val);
  virtual bool enterModeAndWait(int8_t mode);
  virtual bool isModeSupported(int8_t mode);
  virtual int8_t getMode();
  bool readState();
  void handleDiag();

  bool handleAutoSetup();

  /**
   * @brief Initialise the drive
   *
   * This function intialises the drive. This means, it first
   * attempts to bring the device to operational state (CIA402)
   * and then executes the chosen homing method.
   *
   */
  bool handleInit();

  /**
   * @brief Read objects of the drive
   *
   * This function should be called regularly. It reads the status word
   * from the device and translates it into the devices state.
   *
   */
  void handleRead();

  /**
   * @brief Writes objects to the drive
   *
   * This function should be called regularly. It writes the new command
   * word to the drive
   *
   */
  void handleWrite();

  /**
   * @brief Shutdowns the drive
   *
   * This function shuts down the drive by bringing it into
   * SwitchOn disbled state.
   *
   */
  bool handleShutdown();

  /**
   * @brief Executes a quickstop
   *
   * The function executes a quickstop.
   *
   */
  bool handleHalt();

  /**
   * @brief Recovers the device from fault
   *
   * This function tries to reset faults and
   * put the device back to operational state.
   *
   */
  bool handleRecover();

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

  /**
   * @brief Tries to register the standard operation modes defined in cia402
   *
   */
  virtual void registerDefaultModes()
  {
    registerMode<AutoSetupMode>(MotorNanotec::Auto_Setup, driver);
    registerMode<ProfiledPositionMode>(MotorBase::Profiled_Position, driver);
    registerMode<VelocityMode>(MotorBase::Velocity, driver);
    registerMode<ProfiledVelocityMode>(MotorBase::Profiled_Velocity, driver);
    registerMode<ProfiledTorqueMode>(MotorBase::Profiled_Torque, driver);
    registerMode<DefaultHomingMode>(MotorBase::Homing, driver);
    registerMode<InterpolatedPositionMode>(MotorBase::Interpolated_Position, driver);
    registerMode<CyclicSynchronousPositionMode>(MotorBase::Cyclic_Synchronous_Position, driver);
    registerMode<CyclicSynchronousVelocityMode>(MotorBase::Cyclic_Synchronous_Velocity, driver);
    registerMode<CyclicSynchronousTorqueMode>(MotorBase::Cyclic_Synchronous_Torque, driver);
  }

private:
  virtual bool isModeSupportedByDevice(int8_t mode);
  void registerMode(int8_t id, const ModeSharedPtr & m);

  ModeSharedPtr allocMode(int8_t mode);

  bool switchMode(int8_t mode);
  bool switchState(const State402::InternalState & target);

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
  const State402::InternalState switching_state_;
  const bool monitor_mode_;
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

} // namespace aleph2_canopen
