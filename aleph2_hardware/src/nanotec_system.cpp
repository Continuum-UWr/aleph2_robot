#include <limits>
#include <cmath>

#include "aleph2_hardware/nanotec_system.hpp"

#include "rclcpp/logging.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace aleph2_hardware
{
CallbackReturn Aleph2NanotecSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // TODO Read joint configuration
  }

  clock_ = rclcpp::Clock();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Aleph2NanotecSystem::export_state_interfaces()
{
  return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface>
Aleph2NanotecSystem::export_command_interfaces()
{
  return std::vector<hardware_interface::CommandInterface>();
}

hardware_interface::return_type Aleph2NanotecSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Aleph2NanotecSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return hardware_interface::return_type::OK;
}

CallbackReturn Aleph2NanotecSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++) {
    if (std::isnan(hw_states_positions_[i])) {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i])) {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_efforts_[i])) {
      hw_states_efforts_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_efforts_[i])) {
      hw_commands_efforts_[i] = 0;
    }
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("Aleph2NanotecSystem"), "System successfully started! %u",
    control_level_[0]);
  return CallbackReturn::SUCCESS;
}

CallbackReturn Aleph2NanotecSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Aleph2NanotecSystem::read()
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Aleph2NanotecSystem::write()
{
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aleph2_hardware::Aleph2NanotecSystem,
  hardware_interface::SystemInterface)
