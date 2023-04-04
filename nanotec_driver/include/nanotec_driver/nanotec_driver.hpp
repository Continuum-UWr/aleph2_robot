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

#include <memory>

#include "nanotec_driver/node_interfaces/node_canopen_nanotec_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace nanotec_driver
{

class NanotecDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenNanotecDriver> node_canopen_nanotec_driver_;

public:
  explicit NanotecDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  std::shared_ptr<MotorNanotec> get_motor()
  {
    return node_canopen_nanotec_driver_->get_motor();
  }

  bool start_node_nmt_command()
  {
    return node_canopen_nanotec_driver_->start_node_nmt_command();
  }

  bool reset_node_nmt_command()
  {
    return node_canopen_nanotec_driver_->reset_node_nmt_command();
  }
};

}  // namespace nanotec_driver
