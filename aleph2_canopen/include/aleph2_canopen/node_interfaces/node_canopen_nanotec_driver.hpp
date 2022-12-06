#pragma once

#include "aleph2_canopen/motor.hpp"

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_402_driver/lely_motion_controller_bridge.hpp"

namespace aleph2_canopen
{
namespace node_interfaces
{

class NodeCanopenNanotecDriver : public ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>
{
protected:
  std::shared_ptr<LelyMotionControllerBridge> mc_driver_;
  std::shared_ptr<MotorNanotec> motor_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_auto_setup_service;

  uint32_t period_ms_;

  void run();

public:
  NodeCanopenNanotecDriver(rclcpp::Node * node);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;
  virtual void remove_from_master() override;

  void handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_auto_setup(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
};

} // namespace node_interfaces
} // namespace aleph2_canopen
