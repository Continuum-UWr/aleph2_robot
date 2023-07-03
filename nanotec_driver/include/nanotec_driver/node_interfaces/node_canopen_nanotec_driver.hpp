#pragma once

#include <memory>
#include <vector>

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "nanotec_driver/motor.hpp"

namespace nanotec_driver
{
namespace node_interfaces
{

class NodeCanopenNanotecDriver
  : public ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>
{
protected:
  std::shared_ptr<MotorNanotec> motor_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_switch_off_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_switch_enabled_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_switch_operational_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_recover_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_auto_setup_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_mode_position_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_mode_velocity_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_mode_torque_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr srv_set_target_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameter_callback_handle_;

  void publish();

  void handle_switch_off(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_switch_enabled(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_switch_operational(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_auto_setup(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_mode_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response);

  rcl_interfaces::msg::SetParametersResult handle_on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  void on_emcy(ros2_canopen::COEmcy emcy) override;

public:
  explicit NodeCanopenNanotecDriver(rclcpp::Node * node);

  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;
  void poll_timer_callback() override;

  std::shared_ptr<MotorNanotec> get_motor();
};

}  // namespace node_interfaces
}  // namespace nanotec_driver
