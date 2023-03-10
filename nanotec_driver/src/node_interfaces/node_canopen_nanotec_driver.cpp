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

  mc_driver_->validate_objs();

  update_timer_ = this->node_->create_wall_timer(
    std::chrono::milliseconds(period_ms_),
    std::bind(&NodeCanopenNanotecDriver::update, this),
    this->timer_cbg_);

  this->node_->set_parameter(
    rclcpp::Parameter(
      "current_controller_kp",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(current_controller_kp_for_iq_))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "current_controller_ti",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(current_controller_ti_for_iq_))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "velocity_controller_kp",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(velocity_controller_kp_))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "velocity_controller_ti",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(velocity_controller_ti_))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "position_controller_kp",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(position_controller_kp_))));
  this->node_->set_parameter(
    rclcpp::Parameter(
      "position_controller_ti",
      static_cast<int>(this->mc_driver_->get_remote_obj<uint32_t>(position_controller_ti_))));

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

  std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>>> prom;
  prom =
    std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>>>();
  std::future<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>> f = prom->get_future();
  this->exec_->post(
    [this, prom]()
    {
      std::scoped_lock<std::mutex> lock(this->driver_mutex_);
      mc_driver_ =
      std::make_shared<LelyMotionControllerBridge>(
        *(this->exec_), *(this->master_),
        this->node_id_, this->node_->get_name());
      mc_driver_->Boot();
      prom->set_value(mc_driver_);
    });
  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(this->node_->get_logger(), "Adding timed out.");
    throw DriverException("add_to_master: Adding timed out.");
  }
  this->mc_driver_ = f.get();
  this->motor_ =
    std::make_shared<MotorNanotec>(mc_driver_, node_->get_logger().get_child("MotorNanotec"));

  current_controller_kp_for_iq_ = this->mc_driver_->create_remote_obj(
    0x321A, 1U, CODataTypes::COData32);
  current_controller_ti_for_iq_ = this->mc_driver_->create_remote_obj(
    0x321A, 2U, CODataTypes::COData32);
  current_controller_kp_for_id_ = this->mc_driver_->create_remote_obj(
    0x321A, 3U, CODataTypes::COData32);
  current_controller_ti_for_id_ = this->mc_driver_->create_remote_obj(
    0x321A, 4U, CODataTypes::COData32);
  velocity_controller_kp_ = this->mc_driver_->create_remote_obj(0x321B, 1U, CODataTypes::COData32);
  velocity_controller_ti_ = this->mc_driver_->create_remote_obj(0x321B, 2U, CODataTypes::COData32);
  position_controller_kp_ = this->mc_driver_->create_remote_obj(0x321C, 1U, CODataTypes::COData32);
  position_controller_ti_ = this->mc_driver_->create_remote_obj(0x321C, 2U, CODataTypes::COData32);

  this->lely_driver_ = std::static_pointer_cast<LelyDriverBridge>(mc_driver_);
  this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(mc_driver_);
  if (!this->mc_driver_->IsReady()) {
    RCLCPP_WARN(this->node_->get_logger(), "Wait for device to boot.");
    try {
      this->mc_driver_->wait_for_boot();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->node_->get_logger(), e.what());
      std::string msg;
      msg.append("add_to_master: ");
      msg.append(e.what());
      throw DriverException(msg);
    }
  }
  RCLCPP_INFO(this->node_->get_logger(), "Driver booted and ready.");
}

void NodeCanopenNanotecDriver::remove_from_master()
{
  RCLCPP_INFO(node_->get_logger(), "remove_from_master");

  std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
  auto f = prom->get_future();
  this->exec_->post(
    [this, prom]()
    {
      this->motor_.reset();
      this->driver_.reset();
      this->lely_driver_.reset();
      this->mc_driver_.reset();
      prom->set_value();
    });

  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready) {
    throw DriverException("remove_from_master: removing timed out");
  }
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
  response->success = motor_->set_mode(MotorBase::Profiled_Position);
}

void NodeCanopenNanotecDriver::handle_set_mode_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->set_mode(MotorBase::Profiled_Velocity);
}

void NodeCanopenNanotecDriver::handle_set_mode_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  response->success = motor_->set_mode(MotorBase::Profiled_Torque);
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
      this->mc_driver_->set_remote_obj<uint32_t>(
        current_controller_kp_for_id_, (uint32_t)param.as_int());
      this->mc_driver_->set_remote_obj<uint32_t>(
        current_controller_kp_for_iq_, (uint32_t)param.as_int());
    } else if (param.get_name() == "current_controller_ti") {
      this->mc_driver_->set_remote_obj<uint32_t>(
        current_controller_ti_for_id_, (uint32_t)param.as_int());
      this->mc_driver_->set_remote_obj<uint32_t>(
        current_controller_ti_for_iq_, (uint32_t)param.as_int());
    } else if (param.get_name() == "velocity_controller_kp") {
      this->mc_driver_->set_remote_obj<uint32_t>(
        velocity_controller_kp_, (uint32_t)param.as_int());
    } else if (param.get_name() == "velocity_controller_ti") {
      this->mc_driver_->set_remote_obj<uint32_t>(
        velocity_controller_ti_, (uint32_t)param.as_int());
    } else if (param.get_name() == "position_controller_kp") {
      this->mc_driver_->set_remote_obj<uint32_t>(
        position_controller_kp_, (uint32_t)param.as_int());
    } else if (param.get_name() == "position_controller_ti") {
      this->mc_driver_->set_remote_obj<uint32_t>(
        position_controller_ti_, (uint32_t)param.as_int());
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
