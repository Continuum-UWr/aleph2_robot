#ifndef ROS_CONTROL__ALEPH2_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ALEPH2_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/ros.h>


namespace aleph2_hardware_interface
{

    class Aleph2HardwareInterface: public hardware_interface::RobotHW
    {
        public:
            Aleph2HardwareInterface();
            ~Aleph2HardwareInterface();
            void init(ros::NodeHandle& robot_hw_nh);
            void read();
            void write();

        protected:
            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;
            joint_limits_interface::EffortJointSaturationInterface effort_joint_limits_interface_;

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