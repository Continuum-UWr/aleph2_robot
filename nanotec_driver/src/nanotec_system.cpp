#include "nanotec_driver/nanotec_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace nanotec_driver
{

NanotecSystem::NanotecSystem() {}

NanotecSystem::~NanotecSystem() {}

hardware_interface::CallbackReturn
NanotecSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // RCLCPP_INFO(kLogger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  // RCLCPP_INFO(kLogger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  // RCLCPP_INFO(kLogger, "can_interface_name_name: '%s'", info_.hardware_parameters["can_interface_name_name"].c_str());
  // RCLCPP_INFO(kLogger, "master_bin: '%s'", info_.hardware_parameters["master_bin"].c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
NanotecSystem::export_state_interfaces()
{
  return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface>
NanotecSystem::export_command_interfaces()
{
  return std::vector<hardware_interface::CommandInterface>();
}

hardware_interface::CallbackReturn
NanotecSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
NanotecSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
NanotecSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}


} // namespace nanotec_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nanotec_driver::NanotecSystem, hardware_interface::SystemInterface)
