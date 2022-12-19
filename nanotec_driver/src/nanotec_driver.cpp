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
