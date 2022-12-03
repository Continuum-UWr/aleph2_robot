#include "aleph2_canopen/node_interfaces/node_canopen_nanotec_driver.hpp"

using namespace std::placeholders;

namespace aleph2_canopen
{
namespace node_interfaces
{

NodeCanopenNanotecDriver::NodeCanopenNanotecDriver(rclcpp::Node * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>(node)
{
}

void NodeCanopenNanotecDriver::init(bool called_from_base)
{
  RCLCPP_INFO(node_->get_logger(), "init");
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  handle_auto_setup_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/auto_setup", std::bind(&NodeCanopenNanotecDriver::handle_auto_setup, this, _1, _2));
}

void NodeCanopenNanotecDriver::configure(bool called_from_base)
{
  RCLCPP_INFO(node_->get_logger(), "configure");
  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
}

void NodeCanopenNanotecDriver::activate(bool called_from_base)
{
  RCLCPP_INFO(node_->get_logger(), "activate");
  NodeCanopenProxyDriver<rclcpp::Node>::activate(false);
}

void NodeCanopenNanotecDriver::deactivate(bool called_from_base)
{
  RCLCPP_INFO(node_->get_logger(), "deactivate");
  NodeCanopenProxyDriver<rclcpp::Node>::deactivate(false);
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
  this->motor_ = std::make_shared<MotorNanotec>(mc_driver_);
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

void NodeCanopenNanotecDriver::handle_auto_setup(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load()) {
    response->success = this->motor_->handleAutoSetup();
  }
}

} // namespace node_interfaces
} // namespace aleph2_canopen
