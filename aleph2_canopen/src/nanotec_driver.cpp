#include "aleph2_canopen/nanotec_driver.hpp"

namespace aleph2_canopen
{

NanotecDriver::NanotecDriver(rclcpp::NodeOptions node_options)
: CanopenDriver(node_options)
{
  node_canopen_nanotec_driver_ =
    std::make_shared<aleph2_canopen::node_interfaces::NodeCanopenNanotecDriver>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<ros2_canopen::node_interfaces::NodeCanopenDriverInterface>(
    node_canopen_nanotec_driver_);
}

} // namespace aleph2_canopen

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aleph2_canopen::NanotecDriver)
