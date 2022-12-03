#pragma once

#include "aleph2_canopen/node_interfaces/node_canopen_nanotec_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace aleph2_canopen
{

class NanotecDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenNanotecDriver> node_canopen_nanotec_driver_;

public:
  NanotecDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

};

} // namespace aleph2_canopen
