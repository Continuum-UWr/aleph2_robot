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
};

}  // namespace nanotec_driver
