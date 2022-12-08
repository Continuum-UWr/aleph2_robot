#pragma once

#include "aleph2_canopen/motor.hpp"

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_402_driver/lely_motion_controller_bridge.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace aleph2_canopen
{
namespace node_interfaces
{

class NodeCanopenNanotecDriver : public ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>
{
protected:
  std::shared_ptr<LelyMotionControllerBridge> mc_driver_;
  std::shared_ptr<MotorNanotec> motor_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_halt_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_recover_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_auto_setup_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_mode_velocity_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr srv_set_target_;

  uint32_t period_ms_;

  void update();
  void publish();

  void handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_shutdown(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_auto_setup(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response);

public:
  NodeCanopenNanotecDriver(rclcpp::Node * node);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;
  virtual void remove_from_master() override;

  bool motor_init();
  bool motor_shutdown();
  bool motor_halt();
  bool motor_recover();
  bool motor_auto_setup();
  bool motor_set_mode_velocity();
  bool motor_set_target(double target);
};

} // namespace node_interfaces
} // namespace aleph2_canopen
