// Copyright (c) 2023 Team Continuum
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <limits>

#include "canopen_core/device_container.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/executors.hpp"

#include "nanotec_driver/nanotec_driver.hpp"
#include "nanotec_driver/motor.hpp"

namespace nanotec_driver
{

enum class ControlMethod
{
  NONE,
  POSITION,
  VELOCITY,
  EFFORT
};

struct NanotecJoint
{
  std::string name;
  uint8_t node_id;
  double position = std::numeric_limits<double>::quiet_NaN();
  double velocity = std::numeric_limits<double>::quiet_NaN();
  double effort = std::numeric_limits<double>::quiet_NaN();
  double position_cmd = std::numeric_limits<double>::quiet_NaN();
  double velocity_cmd = std::numeric_limits<double>::quiet_NaN();
  double effort_cmd = std::numeric_limits<double>::quiet_NaN();
  ControlMethod control_method = ControlMethod::NONE;
  std::shared_ptr<MotorNanotec> motor;
};

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

  hardware_interface::return_type
  perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

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

private:
  void spin();
  void clean();

  rclcpp::Logger logger_;

  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  std::unique_ptr<std::thread> spin_thread_;

  std::vector<NanotecJoint> joints_;

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
};

}  // namespace nanotec_driver
