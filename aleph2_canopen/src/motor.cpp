#include "aleph2_canopen/motor.hpp"

namespace aleph2_canopen
{

const static std::unordered_map<State402::InternalState, std::string> STATE_TO_STRING = {
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

  registerMode<AutoSetupMode>(MotorNanotec::Auto_Setup, driver);
  // registerMode<ProfiledPositionMode>(MotorBase::Profiled_Position, driver);
  // registerMode<VelocityMode>(MotorBase::Velocity, driver);
  registerMode<ProfiledVelocityMode>(MotorBase::Profiled_Velocity, driver);
  // registerMode<ProfiledTorqueMode>(MotorBase::Profiled_Torque, driver);
  // registerMode<DefaultHomingMode>(MotorBase::Homing, driver);
  // registerMode<InterpolatedPositionMode>(MotorBase::Interpolated_Position, driver);
  // registerMode<CyclicSynchronousPositionMode>(MotorBase::Cyclic_Synchronous_Position, driver);
  // registerMode<CyclicSynchronousVelocityMode>(MotorBase::Cyclic_Synchronous_Velocity, driver);
  // registerMode<CyclicSynchronousTorqueMode>(MotorBase::Cyclic_Synchronous_Torque, driver);
}

bool MotorNanotec::setTarget(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(val);
  }
  return false;
}

bool MotorNanotec::isModeSupported(int8_t mode)
{
  return mode != MotorBase::Homing && allocMode(mode);
}

bool MotorNanotec::enterModeAndWait(int8_t mode)
{
  bool okay = mode != MotorBase::Homing && switchMode(mode);
  return okay;
}

int8_t MotorNanotec::getMode()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_ : (int8_t)MotorBase::No_Mode;
}

bool MotorNanotec::isModeSupportedByDevice(int8_t mode)
{
  if (mode < 0) {return true;}
  if (!supported_drive_modes_->valid) {
    // THROW_EXCEPTION(std::runtime_error("Supported drive modes (object 6502) is not valid"));
  }
  uint32_t supported_modes = driver->get_remote_obj_cached<uint32_t>(supported_drive_modes_);
  bool supported = supported_modes & (1 << (mode - 1));
  bool below_max = mode <= 32;
  bool above_min = mode > 0;
  return below_max && above_min && supported;
}

void MotorNanotec::registerMode(int8_t id, const ModeSharedPtr & m)
{
  std::scoped_lock map_lock(map_mutex_);
  if (m && m->mode_id_ == id) {
    modes_.insert(std::make_pair(id, m));
  }
}

ModeSharedPtr MotorNanotec::allocMode(int8_t mode)
{
  ModeSharedPtr res;
  if (isModeSupportedByDevice(mode)) {
    std::scoped_lock map_lock(map_mutex_);
    std::unordered_map<int8_t, ModeSharedPtr>::iterator it = modes_.find(mode);
    if (it != modes_.end()) {
      res = it->second;
    }
  }
  return res;
}

bool MotorNanotec::switchMode(int8_t mode)
{

  if (mode == MotorBase::No_Mode) {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try { // try to set mode
      driver->set_remote_obj<int8_t>(op_mode_, mode);
    } catch (...) {
    }
    return true;
  }

  ModeSharedPtr next_mode = allocMode(mode);
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

    if (mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode) {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switchState(State402::Switched_On)) {
    return false;
  }

  driver->set_remote_obj<int8_t>(op_mode_, mode);

  bool okay = false;

  {   // wait for switch
    std::unique_lock lock(mode_mutex_);

    std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() +
      std::chrono::seconds(5);

    while (mode_id_ != mode &&
      mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout) {}

    if (mode_id_ == mode) {
      selected_mode_ = next_mode;
      okay = true;
    } else {
      RCLCPP_INFO(logger_, "Mode switch timed out.");
      driver->set_remote_obj<int8_t>(op_mode_, mode_id_);
    }
  }

  if (!switchState(State402::Operation_Enable)) {
    return false;
  }

  return okay;
}

bool MotorNanotec::switchState(const State402::InternalState & target)
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

bool MotorNanotec::readState()
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
  int8_t new_mode;
  new_mode = driver->get_remote_obj<int8_t>(op_mode_display_);

  if (selected_mode_ && selected_mode_->mode_id_ == new_mode) {
    if (!selected_mode_->read(sw)) {

      RCLCPP_INFO(logger_, "Mode handler has error.");
    }
  }
  if (new_mode != mode_id_) {
    RCLCPP_INFO_STREAM(logger_, "New mode detected: " << (int)new_mode);

    mode_id_ = new_mode;
    mode_cond_.notify_all();
  }
  if (selected_mode_ && selected_mode_->mode_id_ != new_mode) {
    RCLCPP_INFO(logger_, "Mode does not match.");
  }

  if (sw & (1 << State402::SW_Internal_limit)) {
    if (old_sw & (1 << State402::SW_Internal_limit)) {
      RCLCPP_INFO(logger_, "Internal limit active");
    } else {
      RCLCPP_INFO(logger_, "Internal limit active");
    }
  }

  return true;
}
void MotorNanotec::handleRead()
{
  readState();
}
void MotorNanotec::handleWrite()
{
  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == mode_id_) {
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

bool MotorNanotec::handleInit()
{
  // Register default modes
  for (std::unordered_map<int8_t, AllocFuncType>::iterator it = mode_allocators_.begin();
    it != mode_allocators_.end(); ++it)
  {
    (it->second)();
  }

  RCLCPP_INFO(logger_, "Init: Read State");
  if (!readState()) {
    RCLCPP_ERROR(logger_, "Could not read motor state");
    return false;
  }
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_ = 0;
    start_fault_reset_ = true;
  }

  RCLCPP_INFO(logger_, "Init: Enable");
  if (!switchState(State402::Ready_To_Switch_On) || !switchState(State402::Switched_On)) {
    RCLCPP_ERROR(logger_, "Could not enable motor");
    return false;
  }

  RCLCPP_INFO(logger_, "Init: Switch no mode");
  if (!switchMode(MotorBase::No_Mode)) {
    RCLCPP_ERROR(logger_, "Could not enter no mode");
    return false;
  }
  return true;
}

bool MotorNanotec::handleShutdown()
{
  switchMode(MotorBase::No_Mode);
  return switchState(State402::Switch_On_Disabled);
}

bool MotorNanotec::handleHalt()
{
  State402::InternalState state = state_handler_.getState();
  std::scoped_lock lock(cw_mutex_);

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault) {
    return false;
  }

  if (state != State402::Operation_Enable) {
    target_state_ = state;
  } else {
    target_state_ = State402::Quick_Stop_Active;
    if (!Command402::setTransition(control_word_, state, State402::Quick_Stop_Active, 0)) {
      RCLCPP_ERROR(logger_, "Could not quick stop");
      return false;
    }
  }
  return true;
}

bool MotorNanotec::handleRecover()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_ && !selected_mode_->start()) {
      RCLCPP_ERROR(logger_, "Could not restart mode.");
      return false;
    }
  }
  if (!switchState(State402::Operation_Enable)) {
    RCLCPP_ERROR(logger_, "Could not enable operation");
    return false;
  }
  return true;
}

bool MotorNanotec::handleAutoSetup()
{
  if (!switchMode(MotorNanotec::Auto_Setup)) {
    RCLCPP_ERROR(logger_, "Failed to switch mode to auto setup");
    return false;
  }

  AutoSetupMode * auto_setup = dynamic_cast<AutoSetupMode *>(selected_mode_.get());
  return auto_setup->executeAutoSetup();
}

} // namespace aleph2_canopen
