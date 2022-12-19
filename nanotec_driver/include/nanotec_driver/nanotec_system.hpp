#pragma once

#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace nanotec_driver
{

class NanotecSystem : public hardware_interface::SystemInterface
{
public:
  NanotecSystem();
  ~NanotecSystem();

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace nanotec_driver
