#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::EffortJointSaturationHandle;
using joint_limits_interface::VelocityJointSaturationHandle;
using joint_limits_interface::VelocityJointSoftLimitsHandle;

namespace aleph2_hardware_interface
{
    Aleph2HardwareInterface::Aleph2HardwareInterface() {}
    Aleph2HardwareInterface::~Aleph2HardwareInterface() {}

    void Aleph2HardwareInterface::init(ros::NodeHandle& robot_hw_nh) {
        XmlRpc::XmlRpcValue hardware;
        robot_hw_nh.getParam("hardware_interface/joints", hardware);
        ROS_ASSERT(hardware.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        num_joints_ = hardware.size();

        // Resize vectors
        joint_names_.resize(num_joints_);
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joints_.resize(num_joints_);

        for( int i = 0; i < num_joints_; ++i )
        {
            ROS_ASSERT(hardware[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);

            ROS_ASSERT(hardware[i].hasMember("name"));
            ROS_ASSERT(hardware[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);

            joint_names_[i] = static_cast<std::string>(hardware[i]["name"]);

            ROS_ASSERT(hardware[i].hasMember("type"));
            ROS_ASSERT(hardware[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);

            if( hardware[i]["type"] == "rubi_stepper" )
            {
                // Create joint state handle
                JointStateHandle joint_state_handle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);

                // Create joint velocity handle
                JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i]);

                // Create joint velocity limits handles
                JointLimits joint_limits;
                SoftJointLimits soft_joint_limits;
                getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
                getSoftJointLimits(joint_names_[i], robot_hw_nh, soft_joint_limits);
                VelocityJointSaturationHandle joint_velocity_saturation_handle(joint_velocity_handle, joint_limits);
                VelocityJointSoftLimitsHandle joint_velocity_soft_limits_handle(joint_velocity_handle, joint_limits, soft_joint_limits);

                // Register handles
                joint_state_interface_.registerHandle(joint_state_handle);
                velocity_joint_interface_.registerHandle(joint_velocity_handle);
                velocity_joint_saturation_interface_.registerHandle(joint_velocity_saturation_handle);
                velocity_joint_soft_limits_interface_.registerHandle(joint_velocity_soft_limits_handle);

                ROS_ASSERT(hardware[i].hasMember("position_topic"));
                ROS_ASSERT(hardware[i]["position_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                ROS_ASSERT(hardware[i].hasMember("velocity_topic"));
                ROS_ASSERT(hardware[i]["velocity_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);

                joints_[i] = new aleph2cpp::RubiStepperJoint(hardware[i]["position_topic"], hardware[i]["velocity_topic"]);
            }
            else
            {
                ROS_ERROR_STREAM("Incorrect joint type: " << hardware[i]["type"]);
                ROS_ASSERT(false);
            }
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&velocity_joint_saturation_interface_);
        registerInterface(&velocity_joint_soft_limits_interface_);
    }

    void Aleph2HardwareInterface::read() 
    {
        for( int i = 0; i < num_joints_; ++i )
        {
            joint_position_[i] = joints_[i]->getPosition();
        }
    }

    void Aleph2HardwareInterface::write(ros::Duration elapsed_time) {
        velocity_joint_saturation_interface_.enforceLimits(elapsed_time);
        velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);

        for( int i = 0; i < num_joints_; ++i )
        {
            ROS_INFO_STREAM("joint name: " << joint_names_[i] << " velocity command: " << joint_velocity_command_[i]);
            joints_[i]->setVelocity(joint_velocity_command_[i]);
        }
    }
}