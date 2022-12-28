#include "nanotec_driver/motor.hpp"

namespace nanotec_driver
{

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
  std::shared_ptr<LelyMotionControllerBridge> driver,
  rclcpp::Logger logger)
: logger_(logger), state_switch_timeout_(5)
{
  this->driver = driver;
  status_word_entry_ =
    driver->create_remote_obj(status_word_entry_index, 0U, CODataTypes::COData16);
  control_word_entry_ = driver->create_remote_obj(
    control_word_entry_index, 0U, CODataTypes::COData16);
  op_mode_display_ = driver->create_remote_obj(op_mode_display_index, 0U, CODataTypes::COData8);
  op_mode_ = driver->create_remote_obj(op_mode_index, 0U, CODataTypes::COData8);
  supported_drive_modes_ = driver->create_remote_obj(
    supported_drive_modes_index, 0U, CODataTypes::COData32);

  register_mode(std::make_shared<AutoSetupMode>(driver));
  register_mode(std::make_shared<ProfiledPositionMode>(driver));
  register_mode(std::make_shared<ProfiledVelocityMode>(driver));
  register_mode(std::make_shared<ProfiledTorqueMode>(driver));
}

bool MotorNanotec::set_target(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(val);
  }
  return false;
}

int8_t MotorNanotec::get_mode_id()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_ : (int8_t)MotorBase::No_Mode;
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

void MotorNanotec::register_mode(const ModeSharedPtr & m)
{
  modes_.insert(std::make_pair(m->mode_id_, m));
}

ModeSharedPtr MotorNanotec::get_mode(int8_t mode)
{
  ModeSharedPtr res;

  std::unordered_map<int8_t, ModeSharedPtr>::iterator it = modes_.find(mode);
  if (it != modes_.end()) {
    res = it->second;
  }

  return res;
}

bool MotorNanotec::switch_mode(int8_t mode_id)
{
  if (mode_id == MotorBase::No_Mode) {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try {  // try to set mode
      driver->set_remote_obj<int8_t>(op_mode_, mode_id);
    } catch (...) {
    }
    return true;
  }

  ModeSharedPtr next_mode = get_mode(mode_id);
  if (!next_mode) {
    RCLCPP_INFO(logger_, "Mode is not supported.");
    return false;
  }

  if (!next_mode->start()) {
    RCLCPP_INFO(logger_, "Could not start mode.");
    return false;
  }

  {   // disable mode handler
    std::scoped_lock lock(mode_mutex_);

    if (current_mode_id_ == mode_id && selected_mode_ && selected_mode_->mode_id_ == mode_id) {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switch_state(State402::Switched_On)) {
    return false;
  }

  driver->set_remote_obj<int8_t>(op_mode_, mode_id);

  bool okay = false;

  {   // wait for switch
    std::unique_lock lock(mode_mutex_);

    std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() +
      std::chrono::seconds(5);

    while (current_mode_id_ != mode_id &&
      mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout) {}

    if (current_mode_id_ == mode_id) {
      selected_mode_ = next_mode;
      okay = true;
    } else {
      RCLCPP_INFO(logger_, "Mode switch timed out.");
      driver->set_remote_obj<int8_t>(op_mode_, current_mode_id_);
    }
  }

  if (!switch_state(State402::Operation_Enable)) {
    return false;
  }

  return okay;
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
      RCLCPP_INFO(logger_, "Could not set transition.");
      return false;
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state)) {
      RCLCPP_INFO(logger_, "Transition timed out.");
      return false;
    }
  }
  return state == target;
}

bool MotorNanotec::read_state()
{
  uint16_t old_sw, sw = driver->get_remote_obj<uint16_t>(status_word_entry_);
  old_sw = status_word_.exchange(sw);

  RCLCPP_DEBUG_STREAM(logger_, "Status Word: " << std::bitset<16>{sw}.to_string());

  State402::InternalState old_state = state_handler_.getState();
  state_handler_.read(sw);
  State402::InternalState new_state = state_handler_.getState();

  if (new_state != old_state) {
    RCLCPP_INFO_STREAM(logger_, "New state detected: " << STATE_TO_STRING.at(new_state));
  }

  std::unique_lock lock(mode_mutex_);
  int8_t new_mode_id;
  new_mode_id = driver->get_remote_obj<int8_t>(op_mode_display_);

  if (selected_mode_ && selected_mode_->mode_id_ == new_mode_id) {
    if (!selected_mode_->read(sw)) {
      RCLCPP_INFO(logger_, "Mode handler has error.");
    }
  }
  if (new_mode_id != current_mode_id_) {
    RCLCPP_INFO_STREAM(logger_, "New mode detected: " << (int)new_mode_id);

    current_mode_id_ = new_mode_id;
    mode_cond_.notify_all();
  }
  if (selected_mode_ && selected_mode_->mode_id_ != new_mode_id) {
    RCLCPP_INFO(logger_, "Mode does not match.");
  }

  if (sw & (1 << State402::SW_Internal_limit)) {
    if (old_sw & (1 << State402::SW_Internal_limit)) {
    } else {
      RCLCPP_INFO(logger_, "Internal limit active");
    }
  }

  return true;
}

void MotorNanotec::read()
{
  read_state();
}

void MotorNanotec::write()
{
  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == current_mode_id_) {
      okay = selected_mode_->write(cwa);
    } else {
      cwa = 0;
    }
    if (okay) {
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }
  if (start_fault_reset_.exchange(false)) {
    RCLCPP_INFO(logger_, "Fault reset");
    this->driver->set_remote_obj<uint16_t>(
      control_word_entry_,
      control_word_ & ~(1 << Command402::CW_Fault_Reset));
  } else {
    RCLCPP_DEBUG_STREAM(logger_, "Control Word: " << std::bitset<16>{control_word_}.to_string());
    this->driver->set_remote_obj<uint16_t>(control_word_entry_, control_word_);
  }
}

bool MotorNanotec::init()
{
  RCLCPP_INFO(logger_, "Init: Read State");
  if (!read_state()) {
    RCLCPP_ERROR(logger_, "Could not read motor state");
    return false;
  }
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_ = 0;
    start_fault_reset_ = true;
  }

  RCLCPP_INFO(logger_, "Init: Enable");
  if (!switch_state(State402::Ready_To_Switch_On) || !switch_state(State402::Switched_On)) {
    RCLCPP_ERROR(logger_, "Could not enable motor");
    return false;
  }

  RCLCPP_INFO(logger_, "Init: Switch no mode");
  if (!switch_mode(MotorBase::No_Mode)) {
    RCLCPP_ERROR(logger_, "Could not enter no mode");
    return false;
  }
  return true;
}

bool MotorNanotec::shutdown()
{
  switch_mode(MotorBase::No_Mode);
  return switch_state(State402::Switch_On_Disabled);
}

bool MotorNanotec::enable_operation()
{
  State402::InternalState state = state_handler_.getState();

  if (state == State402::Operation_Enable) {
    RCLCPP_ERROR(logger_, "Could not enable operation: Already in \"Operation Enabled\" state");
    return false;
  }

  if (state == State402::Fault || state == State402::Fault_Reaction_Active) {
    RCLCPP_ERROR(logger_, "Could not enable operation: Motor is in fault state");
    return false;
  }

  if (current_mode_id_ == MotorBase::No_Mode) {
    RCLCPP_ERROR(logger_, "Could not enable operation: No operation mode selected");
    return false;
  }

  if (!switch_state(State402::Operation_Enable)) {
    RCLCPP_ERROR(logger_, "Could not enable operation: Failed to switch state");
    return false;
  }

  return true;
}

bool MotorNanotec::disable_operation()
{
  State402::InternalState state = state_handler_.getState();

  if (state != State402::Operation_Enable) {
    RCLCPP_ERROR(logger_, "Could not disable operation: Not in \"Operation Enabled\" state");
    return false;
  }

  if (!switch_state(State402::Switched_On)) {
    RCLCPP_ERROR(logger_, "Could not disable operation: Failed to switch state");
    return false;
  }

  return true;
}

bool MotorNanotec::recover()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_ && !selected_mode_->start()) {
      RCLCPP_ERROR(logger_, "Could not restart mode.");
      return false;
    }
  }
  if (!switch_state(State402::Operation_Enable)) {
    RCLCPP_ERROR(logger_, "Could not enable operation");
    return false;
  }
  return true;
}

bool MotorNanotec::auto_setup()
{
  if (!switch_mode(MotorNanotec::Auto_Setup)) {
    RCLCPP_ERROR(logger_, "Failed to switch mode to auto setup");
    return false;
  }

  AutoSetupMode * auto_setup = dynamic_cast<AutoSetupMode *>(selected_mode_.get());
  return auto_setup->execute_auto_setup();
}

}  // namespace nanotec_driver
