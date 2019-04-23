#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::EffortJointSaturationHandle;

namespace aleph2_hardware_interface
{
    Aleph2HardwareInterface::Aleph2HardwareInterface() {}
    Aleph2HardwareInterface::~Aleph2HardwareInterface() {}

    void Aleph2HardwareInterface::init(ros::NodeHandle& robot_hw_nh) {
        // Get joint names
        robot_hw_nh.getParam("hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
            // Create joint state interface
            JointStateHandle joint_state_handle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create effort joint interface
            JointHandle joint_effort_handle(joint_state_handle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(joint_effort_handle);

            // Create effort joint limits interface
            JointLimits joint_limits;
            getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
            EffortJointSaturationHandle joint_effort_limits_handle(joint_effort_handle, joint_limits);
            effort_joint_limits_interface_.registerHandle(joint_effort_limits_handle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&effort_joint_limits_interface_);
    }

    void Aleph2HardwareInterface::read() {
        //TODO
    }

    void Aleph2HardwareInterface::write() {
        //TODO
    }
}