#ifndef ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_HARDWARE_INTERFACE_ALEPH2_HW_H_
#define ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_HARDWARE_INTERFACE_ALEPH2_HW_H_

#include <list>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <aleph2_joint/addon.h>
#include <aleph2_joint/joint.h>

using hardware_interface::ControllerInfo;
using hardware_interface::JointHandle;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;

namespace aleph2_hardware_interface {
enum class JointMode { NONE, EFFORT, VELOCITY, POSITION };

class Aleph2HW : public hardware_interface::RobotHW {
 public:
  Aleph2HW();
  ~Aleph2HW();
  void init(const ros::NodeHandle& robot_hw_nh);
  void doSwitch(const std::list<ControllerInfo>& start_controllers,
                const std::list<ControllerInfo>& stop_controllers) override;
  void read();
  void write(ros::Duration elapsed_time);

 protected:
  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  joint_limits_interface::EffortJointSaturationInterface
      effort_joint_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface
      effort_joint_soft_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface
      velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_soft_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface
      position_joint_saturation_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface
      position_joint_soft_limits_interface_;

  int num_joints_;
  std::vector<std::string> joint_names_;
  std::vector<JointMode> joint_modes_;
  std::vector<aleph2_joint::Joint*> joints_;
  std::map<std::string, kaco::Master*> can_masters_;

  void registerLimitsHandles(const JointHandle& joint_effort_handle,
                             const JointHandle& joint_velocity_handle,
                             const JointHandle& joint_position_handle,
                             const JointLimits& joint_limits,
                             bool has_soft_limits,
                             const SoftJointLimits& joint_soft_limits);

  // Shared memory
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
};

}  // namespace aleph2_hardware_interface

#endif  // ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_HARDWARE_INTERFACE_ALEPH2_HW_H_
