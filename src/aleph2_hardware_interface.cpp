#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace aleph2_hardware_interface
{
    Aleph2HardwareInterface::Aleph2HardwareInterface() {
        // init();
        // controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        // nh_.param("aleph2/hardware_interface/loop_rate", loop_hz_, 0.1);
        // ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        // non_realtime_loop_ = nh_.createTimer(update_freq, &Aleph2HardwareInterface::update, this);
    }

    Aleph2HardwareInterface::~Aleph2HardwareInterface() {

    }

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
            ROS_INFO_STREAM("Joint: " << joint_names_[i]);
            //aleph2cpp::Joint joint = aleph2.getJoint(joint_names_[i]);

             // Create joint state interface`
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits joint_limits;
            SoftJointLimits joint_soft_limits;
            getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
            ROS_INFO_STREAM("Limits: " << joint_limits.min_position << " " << joint_limits.max_position);
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, joint_limits, joint_soft_limits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
    }

    // void Aleph2HardwareInterface::update(const ros::TimerEvent& e) {
    //     elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    //     read();
    //     controller_manager_->update(ros::Time::now(), elapsed_time_);
    //     write(elapsed_time_);
    // }

    void Aleph2HardwareInterface::read() {
        // for (int i = 0; i < num_joints_; i++) {
        //     //joint_position_[i] = aleph2.getJoint(joint_names_[i]).read();
        // }
    }

    void Aleph2HardwareInterface::write() {
        // positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
        // for (int i = 0; i < num_joints_; i++) {
        //     //aleph2.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
        // }
    }
}