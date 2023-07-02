#include <bitset>

#include "nanotec_driver/motor.hpp"
#include "nanotec_driver/auto_setup.hpp"
#include "nanotec_driver/profiled_position_mode.hpp"

namespace nanotec_driver
{

constexpr uint16_t status_word_entry_index = 0x6041;
constexpr uint16_t control_word_entry_index = 0x6040;
constexpr uint16_t op_mode_display_index = 0x6061;
constexpr uint16_t op_mode_index = 0x6060;
constexpr uint16_t position_actual_value_index = 0x6064;
constexpr uint16_t velocity_actual_value_index = 0x606C;
constexpr uint16_t torque_actual_value_index = 0x6077;

static const std::unordered_map<State402::InternalState, std::string> STATE_TO_STRING = {
  {State402::Start, "Start"},
  {State402::Not_Ready_To_Switch_On, "Not ready to switch on"},
  {State402::Switch_On_Disabled, "Switch on disabled"},
  {State402::Ready_To_Switch_On, "Ready to switch on"},
  {State402::Switched_On, "Switched on"},
  {State402::Operation_Enable, "Operation enabled"},
  {State402::Quick_Stop_Active, "Quick stop active"},
  {State402::Fault_Reaction_Active, "Fault reaction active"},
  {State402::Fault, "Fault"},
};

static const std::unordered_map<Mode, std::string> MODE_TO_STRING = {
  {Mode::Auto_Setup, "Auto Setup"},
  {Mode::No_Mode, "No Mode"},
  {Mode::Profiled_Position, "Profile Position"},
  {Mode::Profiled_Velocity, "Profile Velocity"},
  {Mode::Profiled_Torque, "Profile Torque"},
};

static const std::unordered_map<uint8_t, std::string> ERROR_NUMBER_TO_DESCRIPTION = {
  {0, "Watchdog-Reset"},
  {1, "Input voltage (+Ub) too high"},
  {2, "Output current too high"},
  {3, "Input voltage (+Ub) too low"},
  {4, "Error at fieldbus"},
  {6, "NMT master takes too long to send Nodeguarding request"},
  {7, "Sensor 1 (see 3204h): Error through electrical fault or defective hardware"},
  {8, "Sensor 2 (see 3204h): Error through electrical fault or defective hardware"},
  {9, "Sensor 3 (see 3204h): Error through electrical fault or defective hardware"},
  {10, "Warning: Positive limit switch exceeded"},
  {11, "Warning: Negative limit switch exceeded"},
  {12, "Overtemperature error"},
  {13, "The values of object 6065h (Following Error Window) and object 6066h "
    "(Following Error Time Out) were exceeded; a fault was triggered."},
  {14, "Warning: Nonvolatile memory full. The current save process could not be "
    "completed; parts of the data of the save process are lost. Controller must be "
    "restarted for cleanup work."},
  {15, "Motor blocked"},
  {16, "Warning: Nonvolatile memory damaged; controller must be restarted for "
    "cleanup work (all saved objects are reset to default)"},
  {17, "Slave took too long to send PDO messages."},
  {18, "Sensor n (see 3204h), where n is greater than 3: Error through electrical "
    "fault or defective hardware"},
  {19, "PDO not processed due to a length error"},
  {20, "PDO length exceeded"},
  {21, "Warning: Restart the controller to avoid future errors when saving "
    "(nonvolatile memory full/corrupt)."},
  {22, "Rated current must be set (203Bh:01h/6075h)"},
  {23, "Encoder resolution, number of pole pairs and some other values are "
    "incorrect."},
  {24, "Motor current is too high, adjust the PI parameters."},
  {25, "Internal software error, generic"},
  {26, "Current too high at digital output"},
  {27, "Unexpected sync length"},
  {30, "Error in speed monitoring: slippage error too large"},
  {32, "Internal error: Correction factor for reference voltage missing in the OTP"},
  {40, "Warning: Ballast resistor thermally overloaded"},
  {46, "Interlock error: Bit 3 in 60FDh is set to \"0\", the motor may not start (see the "
    "section Interlock function in the chapter Digital inputs)"},
};


MotorNanotec::MotorNanotec(
  std::shared_ptr<ros2_canopen::LelyDriverBridge> driver,
  rclcpp::Logger logger)
: logger_(logger), selected_mode_(Mode::No_Mode), state_switch_timeout_(5)
{
  this->driver = driver;

  register_mode(std::make_shared<AutoSetupMode>(driver));
  register_mode(std::make_shared<ProfiledPositionMode>(driver));
  register_mode(std::make_shared<ProfiledVelocityMode>(driver));
  register_mode(std::make_shared<ProfiledTorqueMode>(driver));
}

bool MotorNanotec::set_target(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_helper_ && selected_mode_helper_->setTarget(val);
  }
  return false;
}

Mode MotorNanotec::get_mode()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_;
}

void MotorNanotec::on_emcy(ros2_canopen::COEmcy emcy)
{
  uint8_t error_number = emcy.msef[0];
  auto error = ERROR_NUMBER_TO_DESCRIPTION.find(error_number);
  if (error == ERROR_NUMBER_TO_DESCRIPTION.end()) {
    RCLCPP_ERROR_STREAM(logger_, "EMCY: Unknown error number: " << +error_number);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "EMCY: " << error->second);
  }
}

void MotorNanotec::register_mode(const std::shared_ptr<ModeHelper> & m)
{
  modes_.insert(std::make_pair(m->mode, m));
}

std::shared_ptr<ModeHelper> MotorNanotec::get_mode_helper(Mode mode)
{
  std::shared_ptr<ModeHelper> res;

  auto it = modes_.find(mode);
  if (it != modes_.end()) {
    res = it->second;
  }

  return res;
}

bool MotorNanotec::set_mode(Mode mode)
{
  RCLCPP_INFO_STREAM(
    logger_, "Setting mode to: " << static_cast<int>(mode) << " (" << MODE_TO_STRING.at(
      mode) << ")");

  if (mode == Mode::No_Mode) {
    std::scoped_lock lock(mode_mutex_);

    State402::InternalState state = state_handler_.getState();

    if (state == State402::Operation_Enable) {switch_enabled();}

    selected_mode_helper_.reset();
    selected_mode_ = mode;
    driver->universal_set_value<int8_t>(op_mode_index, 0, static_cast<int8_t>(mode));
    return true;
  }

  std::shared_ptr<ModeHelper> next_mode_helper = get_mode_helper(mode);
  if (!next_mode_helper) {
    RCLCPP_ERROR(logger_, "Could not set mode: Mode is not supported");
    return false;
  }

  if (!next_mode_helper->start()) {
    RCLCPP_ERROR(logger_, "Could not set mode: Failed to start mode");
    return false;
  }

  {
    std::unique_lock lock(mode_mutex_);

    if (current_mode_ == mode && selected_mode_ == mode) {
      // nothing to do
      return true;
    }

    selected_mode_ = mode;
    selected_mode_helper_ = next_mode_helper;

    driver->universal_set_value<int8_t>(op_mode_index, 0, static_cast<int8_t>(mode));

    std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() +
      std::chrono::seconds(5);

    while (current_mode_ != mode &&
      mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout) {}

    if (current_mode_ != mode) {
      RCLCPP_ERROR(logger_, "Could not set mode: Timed out");
      driver->universal_set_value<int8_t>(op_mode_index, 0, static_cast<int8_t>(current_mode_));
      return false;
    }
  }

  return true;
}

bool MotorNanotec::switch_state(const State402::InternalState & target)
{
  std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() +
    state_switch_timeout_;
  State402::InternalState state = state_handler_.getState();
  target_state_ = target;
  while (state != target_state_) {
    std::unique_lock lock(cw_mutex_);
    State402::InternalState next = State402::Unknown;
    bool success = Command402::setTransition(control_word_, state, target_state_, &next);
    if (!success) {
      RCLCPP_ERROR(logger_, "Could not switch state: Failed to set transition.");
      return false;
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state)) {
      RCLCPP_ERROR(logger_, "Could not switch state: Transition timed out.");
      return false;
    }
  }
  return state == target;
}

void MotorNanotec::read()
{
  uint16_t old_sw, sw = driver->universal_get_value<uint16_t>(status_word_entry_index, 0);
  old_sw = status_word_.exchange(sw);

  RCLCPP_DEBUG_STREAM(logger_, "Status Word: " << std::bitset<16>{sw}.to_string());

  State402::InternalState old_state = state_handler_.getState();
  state_handler_.read(sw);
  State402::InternalState new_state = state_handler_.getState();

  if (new_state != old_state) {
    RCLCPP_INFO_STREAM(logger_, "New state detected: " << STATE_TO_STRING.at(new_state));
  }

  {
    std::unique_lock lock(mode_mutex_);

    Mode new_mode =
      static_cast<Mode>(driver->universal_get_value<int8_t>(op_mode_display_index, 0));

    if (selected_mode_ == new_mode && selected_mode_ != Mode::No_Mode) {
      if (!selected_mode_helper_->read(sw)) {
        RCLCPP_ERROR(logger_, "Mode handler read error");
      }
    }

    if (new_mode != current_mode_) {
      std::string mode_str;
      if (MODE_TO_STRING.find(new_mode) == MODE_TO_STRING.end()) {
        mode_str = "UNKNOWN";
      } else {
        mode_str = MODE_TO_STRING.at(new_mode);
      }
      RCLCPP_INFO_STREAM(
        logger_,
        "New mode detected: " << static_cast<int>(new_mode) << " (" << mode_str << ")");

      current_mode_ = new_mode;
      mode_cond_.notify_all();

      if (selected_mode_ != current_mode_) {
        RCLCPP_WARN_STREAM(
          logger_, "Mode does not match with the selected: " <<
            static_cast<int8_t>(selected_mode_) << " (" << MODE_TO_STRING.at(selected_mode_) <<
            ")");
      }
    }
  }

  if ((sw & (1 << State402::SW_Internal_limit)) && !(old_sw & (1 << State402::SW_Internal_limit))) {
    RCLCPP_WARN(logger_, "Internal limit active");
  }

  position_ =
    static_cast<double>(driver->universal_get_value<int32_t>(position_actual_value_index, 0));
  velocity_ =
    static_cast<double>(driver->universal_get_value<int32_t>(velocity_actual_value_index, 0));
  torque_ = static_cast<double>(driver->universal_get_value<int16_t>(torque_actual_value_index, 0));
}

void MotorNanotec::write()
{
  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    ModeHelper::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_helper_ && selected_mode_ == current_mode_) {
      okay = selected_mode_helper_->write(cwa);
    } else {
      cwa = 0;
    }
    if (okay) {
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }

  uint16_t control_word_to_set = control_word_;
  if (start_fault_reset_.exchange(false)) {
    RCLCPP_INFO(logger_, "Fault reset");
    control_word_to_set &= ~(1 << Command402::CW_Fault_Reset);
  }

  this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0, control_word_);

  RCLCPP_DEBUG_STREAM(
    logger_, "Control Word: " << std::bitset<16>{control_word_to_set}.to_string());
}

bool MotorNanotec::switch_off()
{
  start_fault_reset_ = true;

  if (!switch_state(State402::Switch_On_Disabled)) {
    RCLCPP_ERROR(logger_, "Could not switch motor off: Failed to switch state");
    return false;
  }

  return true;
}

bool MotorNanotec::switch_enabled()
{
  State402::InternalState state = state_handler_.getState();

  if (state == State402::Switched_On) {
    RCLCPP_WARN(logger_, "Motor already enabled");
    return true;
  }

  if (state == State402::Fault || state == State402::Fault_Reaction_Active) {
    RCLCPP_ERROR(logger_, "Could not enable motor: Motor is in fault state");
    return false;
  }

  if (state == State402::Switch_On_Disabled && !switch_state(State402::Ready_To_Switch_On)) {
    RCLCPP_ERROR(
      logger_,
      "Could not enable motor: Failed to switch to \"Ready To Switch On\" state");
    return false;
  }

  if (!switch_state(State402::Switched_On)) {
    RCLCPP_ERROR(logger_, "Could not enable motor: Failed to switch to \"Switched On\" state");
    return false;
  }

  return true;
}

bool MotorNanotec::switch_operational()
{
  State402::InternalState state = state_handler_.getState();

  if (state == State402::Operation_Enable) {
    RCLCPP_WARN(logger_, "Motor already in operational state");
    return true;
  }

  if (state == State402::Fault || state == State402::Fault_Reaction_Active) {
    RCLCPP_ERROR(logger_, "Could not enable operation: Motor is in fault state");
    return false;
  }

  if (!selected_mode_helper_) {
    RCLCPP_ERROR(logger_, "Could not enable operation: No operation mode selected");
    return false;
  }

  if (!switch_state(State402::Operation_Enable)) {
    RCLCPP_ERROR(logger_, "Could not enable operation: Failed to switch state");
    return false;
  }

  return true;
}

bool MotorNanotec::recover()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_helper_ && !selected_mode_helper_->start()) {
      RCLCPP_ERROR(logger_, "Could not recover from fault: Failed to restart mode");
      return false;
    }
  }

  if (!switch_state(target_state_)) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Could not recover from fault: Failed to switch to \"" <<
        STATE_TO_STRING.at(target_state_) << "\" state");
    return false;
  }

  return true;
}

bool MotorNanotec::auto_setup()
{
  if (!set_mode(Mode::Auto_Setup)) {
    RCLCPP_ERROR(logger_, "Failed to set mode to auto setup");
    return false;
  }

  if (!switch_operational()) {
    return false;
  }

  AutoSetupMode * auto_setup = dynamic_cast<AutoSetupMode *>(selected_mode_helper_.get());
  return auto_setup->execute_auto_setup() && switch_enabled() && set_mode(Mode::No_Mode);
}

}  // namespace nanotec_driver
