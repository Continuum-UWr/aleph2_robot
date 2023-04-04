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

#include "nanotec_driver/nanotec_driver.hpp"

namespace nanotec_driver
{

NanotecDriver::NanotecDriver(rclcpp::NodeOptions node_options)
: CanopenDriver(node_options)
{
  node_canopen_nanotec_driver_ =
    std::make_shared<nanotec_driver::node_interfaces::NodeCanopenNanotecDriver>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<ros2_canopen::node_interfaces::NodeCanopenDriverInterface>(
    node_canopen_nanotec_driver_);
}

}  // namespace nanotec_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nanotec_driver::NanotecDriver)
