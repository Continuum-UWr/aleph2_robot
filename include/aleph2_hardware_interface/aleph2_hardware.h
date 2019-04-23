#ifndef ROS_CONTROL__ALEPH2_HARDWARE_H
#define ROS_CONTROL__ALEPH2_HARDWARE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/ros.h>

namespace aleph2_hardware_interface
{
    class Aleph2Hardware
    {
        protected:
            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;

            joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
            joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
            joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
            joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
            joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
            joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

            // Custom or available transmissions
            // transmission_interface::RRBOTTransmission rrbot_trans_;
            // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

            // Shared memory
            int num_joints_;
            int joint_mode_; // position, velocity, or effort
            std::vector<std::string> joint_names_;
            std::vector<int> joint_types_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_effort_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_effort_command_;
            std::vector<double> joint_lower_limits_;
            std::vector<double> joint_upper_limits_;
            std::vector<double> joint_effort_limits_;

    };

}

#endif