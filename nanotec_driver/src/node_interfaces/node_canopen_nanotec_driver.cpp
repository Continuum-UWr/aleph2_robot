#include "nanotec_driver/node_interfaces/node_canopen_nanotec_driver.hpp"

using namespace std::placeholders;

namespace nanotec_driver
{
namespace node_interfaces
{

NodeCanopenNanotecDriver::NodeCanopenNanotecDriver(rclcpp::Node * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>(node)
{
}

void NodeCanopenNanotecDriver::init(bool called_from_base)
{
  (void)called_from_base;
  RCLCPP_INFO(node_->get_logger(), "init");

  NodeCanopenProxyDriver<rclcpp::Node>::init(false);

  pub_joint_state_ =
    this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
  srv_switch_off_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/switch_off", std::bind(&NodeCanopenNanotecDriver::handle_switch_off, this, _1, _2));
  srv_switch_enabled_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/switch_enabled",
    std::bind(&NodeCanopenNanotecDriver::handle_switch_enabled, this, _1, _2));
  srv_switch_operational_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/switch_operational",
    std::bind(&NodeCanopenNanotecDriver::handle_switch_operational, this, _1, _2));
  srv_recover_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/recover", std::bind(&NodeCanopenNanotecDriver::handle_recover, this, _1, _2));
  srv_auto_setup_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/auto_setup", std::bind(&NodeCanopenNanotecDriver::handle_auto_setup, this, _1, _2));
  srv_set_mode_position_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/set_mode_position",
    std::bind(&NodeCanopenNanotecDriver::handle_set_mode_position, this, _1, _2));
  srv_set_mode_velocity_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/set_mode_velocity",
    std::bind(&NodeCanopenNanotecDriver::handle_set_mode_velocity, this, _1, _2));
  srv_set_mode_torque_ = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/set_mode_torque",
    std::bind(&NodeCanopenNanotecDriver::handle_set_mode_torque, this, _1, _2));
  srv_set_target_ = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
    "~/set_target", std::bind(&NodeCanopenNanotecDriver::handle_set_target, this, _1, _2));

  this->node_->declare_parameter("current_controller_kp", 0);
  this->node_->declare_parameter("current_controller_ti", 0);
  this->node_->declare_parameter("velocity_controller_kp", 0);
  this->node_->declare_parameter("velocity_controller_ti", 0);
  this->node_->declare_parameter("position_controller_kp", 0);
  this->node_->declare_parameter("position_controller_ti", 0);
}

void NodeCanopenNanotecDriver::configure(bool called_from_base)
{
  (void)called_from_base;
  RCLCPP_INFO(node_->get_logger(), "configure");

  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);

  period_ms_ = this->config_["period"].as<uint32_t>();
}

void NodeCanopenNanotecDriver::activate(bool called_from_base)
{
  (void)called_from_base;
  RCLCPP_INFO(node_->get_logger(), "activate");

  NodeCanopenProxyDriver<rclcpp::Node>::activate(false);

  update_timer_ = this->node_->create_wall_timer(
    std::chrono::milliseconds(period_ms_),
    std::bind(&NodeCanopenNanotecDriver::update, this),
    this->timer_cbg_);

  this->node_->set_parameter(
    rclcpp::Parameter(
      "current_controller_kp",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321A, 1))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "current_controller_ti",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321A, 2))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "velocity_controller_kp",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321B, 1))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "velocity_controller_ti",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321B, 2))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "position_controller_kp",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321C, 1))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "position_controller_ti",
      static_cast<int>(this->lely_driver_->universal_get_value<uint32_t>(0x321C, 2))));

  on_set_parameter_callback_handle_ =
    this->node_->add_on_set_parameters_callback(
    std::bind(
      &NodeCanopenNanotecDriver::
      handle_on_set_parameters, this, std::placeholders::_1));
}

void NodeCanopenNanotecDriver::deactivate(bool called_from_base)
{
  (void)called_from_base;
  RCLCPP_INFO(node_->get_logger(), "deactivate");

  NodeCanopenProxyDriver<rclcpp::Node>::deactivate(false);

  on_set_parameter_callback_handle_.reset();

  this->motor_->switch_off();

  update_timer_->cancel();
}

void NodeCanopenNanotecDriver::update()
{
  motor_->read();
  motor_->write();
  publish();
}

void NodeCanopenNanotecDriver::publish()
{
  sensor_msgs::msg::JointState js_msg;
  js_msg.header.stamp = this->node_->get_clock()->now();
  js_msg.name.push_back(this->node_->get_name());
  js_msg.position.push_back(this->motor_->get_position());
  js_msg.velocity.push_back(this->motor_->get_velocity());
  js_msg.effort.push_back(this->motor_->get_torque());
  pub_joint_state_->publish(js_msg);
}

void NodeCanopenNanotecDriver::add_to_master()
{
  RCLCPP_INFO(node_->get_logger(), "add_to_master");
  NodeCanopenProxyDriver<rclcpp::Node>::add_to_master();
  this->motor_ =
    std::make_shared<MotorNanotec>(
    this->lely_driver_,
    node_->get_logger().get_child("MotorNanotec"));
}

void NodeCanopenNanotecDriver::on_emcy(ros2_canopen::COEmcy emcy)
{
  this->motor_->on_emcy(emcy);
}

void NodeCanopenNanotecDriver::handle_switch_off(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->switch_off();
}

void NodeCanopenNanotecDriver::handle_switch_enabled(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->switch_enabled();
}

void NodeCanopenNanotecDriver::handle_switch_operational(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->switch_operational();
}

void NodeCanopenNanotecDriver::handle_recover(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->recover();
}

void NodeCanopenNanotecDriver::handle_auto_setup(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->auto_setup();
}

void NodeCanopenNanotecDriver::handle_set_mode_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->set_mode(Mode::Profiled_Position);
}

void NodeCanopenNanotecDriver::handle_set_mode_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->set_mode(Mode::Profiled_Velocity);
}

void NodeCanopenNanotecDriver::handle_set_mode_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->set_mode(Mode::Profiled_Torque);
}

void NodeCanopenNanotecDriver::handle_set_target(
  const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
  canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
  response->success = motor_->set_target(request->target);
}

rcl_interfaces::msg::SetParametersResult NodeCanopenNanotecDriver::handle_on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto & param : parameters) {
    if (param.get_name() == "current_controller_kp") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321A, 1, (uint32_t)param.as_int());
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321A, 3, (uint32_t)param.as_int());
    } else if (param.get_name() == "current_controller_ti") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321A, 2, (uint32_t)param.as_int());
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321A, 4, (uint32_t)param.as_int());
    } else if (param.get_name() == "velocity_controller_kp") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321B, 1, (uint32_t)param.as_int());
    } else if (param.get_name() == "velocity_controller_ti") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321B, 2, (uint32_t)param.as_int());
    } else if (param.get_name() == "position_controller_kp") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321C, 1, (uint32_t)param.as_int());
    } else if (param.get_name() == "position_controller_ti") {
      this->lely_driver_->universal_set_value<uint32_t>(
        0x321C, 2, (uint32_t)param.as_int());
    }

    RCLCPP_INFO_STREAM(
      this->node_->get_logger(),
      "Parameter \"" << param.get_name() << "\" changed to " << param.value_to_string());
  }

  return result;
}

std::shared_ptr<MotorNanotec> NodeCanopenNanotecDriver::get_motor()
{
  return motor_;
}

}  // namespace node_interfaces
}  // namespace nanotec_driver
