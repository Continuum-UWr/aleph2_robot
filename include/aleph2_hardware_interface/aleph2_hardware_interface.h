#ifndef ALEPH2_HARDWARE_INTERFACE_H
#define ALEPH2_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/ros.h>
#include <aleph2_joint/joint.h>
#include <aleph2_joint/addon.h>

using namespace hardware_interface;
using namespace joint_limits_interface;

namespace aleph2_hardware_interface
{

    class Aleph2HardwareInterface: public hardware_interface::RobotHW
    {
        public:
            Aleph2HardwareInterface();
            ~Aleph2HardwareInterface();
            void init(ros::NodeHandle& robot_hw_nh);
            void read();
            void write(ros::Duration elapsed_time);

        protected:
            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;
            joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
            joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_soft_limits_interface_;
            joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
            joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_soft_limits_interface_;
            joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
            joint_limits_interface::PositionJointSoftLimitsInterface position_joint_soft_limits_interface_;

            // Shared memory
            int num_joints_;
            std::vector<std::string> joint_names_;
            std::vector<int> joint_types_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_effort_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_effort_command_;
            std::vector<aleph2_joint::Joint*> joints_;

            void registerLimitsHandles(JointHandle& joint_effort_handle, JointHandle& joint_velocity_handle, JointHandle& joint_position_handle, 
                JointLimits& joint_limits, bool has_soft_limits, SoftJointLimits& joint_soft_limits);
    };

}

#endif