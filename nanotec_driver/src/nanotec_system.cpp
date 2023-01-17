#include <filesystem>

#include "nanotec_driver/nanotec_system.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace fs = std::filesystem;

namespace nanotec_driver
{

NanotecSystem::NanotecSystem()
: logger_(rclcpp::get_logger("NanotecSystem")) {}

NanotecSystem::~NanotecSystem() {clean();}

void NanotecSystem::clean()
{
  init_thread_->join();
  init_thread_.reset();

  executor_->cancel();
  spin_thread_->join();

  device_container_.reset();
  executor_.reset();

  executor_.reset();
  spin_thread_.reset();
}

hardware_interface::CallbackReturn
NanotecSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger(info.name);

  joints_.resize(info.joints.size());

  for (size_t i = 0; i < info.joints.size(); i++) {
    auto & joint_info = info.joints[i];
    std::string joint_name = joints_[i].name = joint_info.name;

    if (joint_info.parameters.find("node_id") == joint_info.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "Joint \"" << joint_name << "\" missing \"node_id\" parameter");
      return CallbackReturn::FAILURE;
    }

    joints_[i].node_id = static_cast<uint8_t>(std::stoi(joint_info.parameters.at("node_id")));

    state_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_POSITION, &joints_[i].position);
    state_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity);
    state_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_EFFORT, &joints_[i].effort);
    command_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_POSITION, &joints_[i].position_cmd);
    command_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity_cmd);
    command_interfaces_.emplace_back(
      joint_name, hardware_interface::HW_IF_EFFORT, &joints_[i].effort_cmd);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_configure(const rclcpp_lifecycle::State &)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
  spin_thread_ = std::make_unique<std::thread>(&NanotecSystem::spin, this);
  init_thread_ = std::make_unique<std::thread>(&NanotecSystem::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable()) {
    init_thread_->join();
  } else {
    RCLCPP_ERROR(logger_, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    if (!joint.motor) {
      RCLCPP_ERROR(logger_, "Missing driver for joint: '%s'", joint.name.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  return CallbackReturn::SUCCESS;
}

void NanotecSystem::spin()
{
  executor_->spin();
  executor_->remove_node(device_container_);

  RCLCPP_INFO(logger_, "Exiting spin thread...");
}

void NanotecSystem::initDeviceContainer()
{
  std::string bus_config_package = info_.hardware_parameters["bus_config_package"];

  fs::path bus_config_path =
    fs::path(ament_index_cpp::get_package_share_directory(bus_config_package)) /
    fs::path("config") /
    fs::path(info_.hardware_parameters["bus_config_name"]);

  device_container_->init(
    info_.hardware_parameters["can_interface_name"],
    bus_config_path / fs::path("master.dcf"),
    bus_config_path / fs::path("bus.yml"),
    bus_config_path / fs::path("master.bin"));

  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(logger_, "Number of registered drivers: '%zu'", device_container_->count_drivers());
  for (auto it = drivers.begin(); it != drivers.end(); it++) {
    auto nanotec_driver = std::static_pointer_cast<NanotecDriver>(it->second);
    uint8_t node_id = static_cast<uint8_t>(it->first);

    RCLCPP_INFO(
      logger_, "\nRegistered driver:\n    name: '%s'\n    node_id: '%u'",
      nanotec_driver->get_node_base_interface()->get_name(), node_id);

    bool joint_found = false;
    for (auto & joint : joints_) {
      if (joint.node_id == node_id) {
        joint_found = true;
        joint.motor = nanotec_driver->get_motor();
        break;
      }
    }

    if (!joint_found) {
      RCLCPP_WARN(logger_, "No joint specified for node_id: '%u'", node_id);
    }
  }

  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

std::vector<hardware_interface::StateInterface>
NanotecSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
NanotecSystem::export_command_interfaces()
{
  return std::move(command_interfaces_);
}

hardware_interface::CallbackReturn
NanotecSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  clean();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  clean();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
NanotecSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
NanotecSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_) {
    joint.position = joint.motor->get_position();
    joint.velocity = joint.motor->get_velocity();
    joint.effort = joint.motor->get_torque();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
NanotecSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}


}  // namespace nanotec_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nanotec_driver::NanotecSystem, hardware_interface::SystemInterface)
