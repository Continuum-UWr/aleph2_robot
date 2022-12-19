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
};

}  // namespace nanotec_driver
