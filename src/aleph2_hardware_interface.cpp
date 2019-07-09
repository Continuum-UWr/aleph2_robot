#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <nanotec_driver/nanotec.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSaturationHandle;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::PositionJointSoftLimitsHandle;

namespace aleph2_hardware_interface
{
    Aleph2HardwareInterface::Aleph2HardwareInterface() {}
    Aleph2HardwareInterface::~Aleph2HardwareInterface() {}

    void Aleph2HardwareInterface::init(ros::NodeHandle& robot_hw_nh) {
        XmlRpc::XmlRpcValue hardware;
        robot_hw_nh.getParam("joint", hardware);

        ROS_ASSERT(hardware.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        
        num_joints_ = hardware.size();

        // Resize vectors
        joint_names_.resize(num_joints_);
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);
        joints_.resize(num_joints_);

        int i = 0;
        for( auto const& joint : hardware )
        {
            joint_names_[i] = joint.first;
            auto joint_struct = joint.second;
            
            ROS_ASSERT(joint_struct.getType() == XmlRpc::XmlRpcValue::TypeStruct);

            ROS_ASSERT(joint_struct.hasMember("type"));
            ROS_ASSERT(joint_struct["type"].getType() == XmlRpc::XmlRpcValue::TypeString);

            if( joint_struct["type"] == "rubi_stepper" )
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

                ROS_ASSERT(joint_struct.hasMember("position_topic"));
                ROS_ASSERT(joint_struct["position_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                ROS_ASSERT(joint_struct.hasMember("velocity_topic"));
                ROS_ASSERT(joint_struct["velocity_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
                ROS_ASSERT(joint_struct.hasMember("scale"));
                ROS_ASSERT(joint_struct["scale"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

                joints_[i] = new aleph2cpp::RubiStepperJoint(
                    joint_struct["position_topic"], 
                    joint_struct["velocity_topic"],
                    static_cast<double>(joint_struct["scale"])
                );
            }
            else if( joint_struct["type"] == "nanotec" )
            {
                // Create joint state handle
                JointStateHandle joint_state_handle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);

                // Create joint velocity handle
                JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i]);

                // Create joint position handle
                JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i]);

                // Create joint effort handle
                JointHandle joint_effort_handle(joint_state_handle, &joint_effort_command_[i]);

                // Create joint velocity limits handles
                JointLimits joint_limits;
                SoftJointLimits soft_joint_limits;
                getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
                getSoftJointLimits(joint_names_[i], robot_hw_nh, soft_joint_limits);
                VelocityJointSaturationHandle joint_velocity_saturation_handle(joint_velocity_handle, joint_limits);
                VelocityJointSoftLimitsHandle joint_velocity_soft_limits_handle(joint_velocity_handle, joint_limits, soft_joint_limits);

                // Create joint position limits handles
                PositionJointSaturationHandle joint_position_saturation_handle(joint_position_handle, joint_limits);
                PositionJointSoftLimitsHandle joint_position_soft_limits_handle(joint_position_handle, joint_limits, soft_joint_limits);

                // Register handles
                joint_state_interface_.registerHandle(joint_state_handle);
                velocity_joint_interface_.registerHandle(joint_velocity_handle);
                position_joint_interface_.registerHandle(joint_position_handle);
                effort_joint_interface_.registerHandle(joint_effort_handle);
                velocity_joint_saturation_interface_.registerHandle(joint_velocity_saturation_handle);
                velocity_joint_soft_limits_interface_.registerHandle(joint_velocity_soft_limits_handle);
                position_joint_saturation_interface_.registerHandle(joint_position_saturation_handle);
                position_joint_soft_limits_interface_.registerHandle(joint_position_soft_limits_handle);

                ROS_ASSERT(joint_struct.hasMember("node_id"));
                ROS_ASSERT(joint_struct["node_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(joint_struct.hasMember("busname"));
                ROS_ASSERT(joint_struct["busname"].getType() == XmlRpc::XmlRpcValue::TypeString);
                ROS_ASSERT(joint_struct.hasMember("baudrate"));
                ROS_ASSERT(joint_struct["baudrate"].getType() == XmlRpc::XmlRpcValue::TypeString);

                joints_[i] = new aleph2cpp::NanotecJoint(
                    static_cast<int>(joint_struct["node_id"]),
                    joint_struct["busname"],
                    joint_struct["baudrate"],
                    Nanotec::OperationMode::VELOCITY
                );
            }
            else
            {
                ROS_ERROR_STREAM("Incorrect joint type: " << joint_struct["type"]);
                ROS_ASSERT(false);
            }
            ++i;
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&velocity_joint_saturation_interface_);
        registerInterface(&velocity_joint_soft_limits_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&position_joint_saturation_interface_);
        registerInterface(&position_joint_soft_limits_interface_);
        registerInterface(&effort_joint_interface_);
    }

    void Aleph2HardwareInterface::read() 
    {
        for( int i = 0; i < num_joints_; ++i )
        {
            try {
                aleph2cpp::JointType type = joints_[i]->getType();
                switch(type)
                {
                case aleph2cpp::JointType::RUBI_STEPPER:
                    joint_velocity_[i] = joints_[i]->getVelocity();
                    joint_position_[i] = joints_[i]->getPosition();
                    break;
                case aleph2cpp::JointType::NANOTEC:
                    joint_effort_[i] = joints_[i]->getEffort();
                    joint_velocity_[i] = joints_[i]->getVelocity();
                    joint_position_[i] = joints_[i]->getPosition();
                    break;
                }
            } catch (const char* msg) {
                ROS_ERROR_STREAM(msg);
            }
        }
    }

    void Aleph2HardwareInterface::write(ros::Duration elapsed_time) {
        velocity_joint_saturation_interface_.enforceLimits(elapsed_time);
        velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);
        position_joint_saturation_interface_.enforceLimits(elapsed_time);
        position_joint_soft_limits_interface_.enforceLimits(elapsed_time);

        for( int i = 0; i < num_joints_; ++i )
        {
            try {
                aleph2cpp::JointType type = joints_[i]->getType();
                switch(type)
                {
                case aleph2cpp::JointType::RUBI_STEPPER:
                    joints_[i]->setVelocity(joint_velocity_command_[i]);
                    break;
                case aleph2cpp::JointType::NANOTEC:
                    //joints_[i]->setEffort(joint_effort_command_[i]);
                    joints_[i]->setVelocity(joint_velocity_command_[i]);
                    break;
                }
            } catch (const char* msg) {
                ROS_ERROR_STREAM(msg);
            }
            
        }
    }
}