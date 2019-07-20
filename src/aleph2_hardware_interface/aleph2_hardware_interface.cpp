#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <aleph2_hardware_interface/utils.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <nanotec_driver/nanotec.h>
#include <thread>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSaturationHandle;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using XmlRpc::XmlRpcValue;

namespace aleph2_hardware_interface
{
    Aleph2HardwareInterface::Aleph2HardwareInterface() {}
    Aleph2HardwareInterface::~Aleph2HardwareInterface() {}

    void Aleph2HardwareInterface::init(ros::NodeHandle& robot_hw_nh) {
        XmlRpcValue hardware, nanotec_presets;
        robot_hw_nh.getParam("joints", hardware);
        robot_hw_nh.getParam("nanotec_presets", nanotec_presets);

        ROS_ASSERT(hardware.getType() == XmlRpcValue::TypeStruct);
        ROS_ASSERT(nanotec_presets.getType() == XmlRpcValue::TypeStruct || !nanotec_presets.valid());

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

        std::thread joint_init_thread[num_joints_];

        int i = 0;
        for( auto const& joint : hardware )
        {
            joint_names_[i] = joint.first;
            joint_init_thread[i] = std::thread(
                [this, &robot_hw_nh, &nanotec_presets](XmlRpcValue joint_struct, int i) 
            {
                ROS_ASSERT(joint_struct.getType() == XmlRpcValue::TypeStruct);
                ROS_ASSERT(joint_struct.hasMember("type"));
                ROS_ASSERT(joint_struct["type"].getType() == XmlRpcValue::TypeString);

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
                    ROS_ASSERT(joint_struct["position_topic"].getType() == XmlRpcValue::TypeString);
                    ROS_ASSERT(joint_struct.hasMember("velocity_topic"));
                    ROS_ASSERT(joint_struct["velocity_topic"].getType() == XmlRpcValue::TypeString);
                    ROS_ASSERT(joint_struct.hasMember("scale"));
                    ROS_ASSERT(joint_struct["scale"].getType() == XmlRpcValue::TypeDouble);

                    joints_[i] = new aleph2_joint::RubiStepperJoint(
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
                    ROS_ASSERT(joint_struct["node_id"].getType() == XmlRpcValue::TypeInt);
                    ROS_ASSERT(joint_struct.hasMember("busname"));
                    ROS_ASSERT(joint_struct["busname"].getType() == XmlRpcValue::TypeString);
                    ROS_ASSERT(joint_struct.hasMember("baudrate"));
                    ROS_ASSERT(joint_struct["baudrate"].getType() == XmlRpcValue::TypeString);
                    ROS_ASSERT(joint_struct.hasMember("scale"));
                    ROS_ASSERT(joint_struct["scale"].getType() == XmlRpcValue::TypeDouble);

                    std::map<std::string, int64_t> parameters;

                    if (joint_struct.hasMember("preset"))
                    {
                        ROS_ASSERT(joint_struct["preset"].getType() == XmlRpcValue::TypeString);

                        std::string preset = joint_struct["preset"];
                        if (nanotec_presets.valid() && nanotec_presets.hasMember(preset))
                        {
                            ROS_ASSERT(nanotec_presets[preset].getType() == XmlRpcValue::TypeStruct);
                            LoadNanotecParametersFromStruct(parameters, nanotec_presets[preset]);
                        }
                        else 
                        {
                            ROS_ERROR_STREAM("Nanotec preset " << preset << " is not defined");
                        }
                    }

                    if (joint_struct.hasMember("parameters"))
                    {
                        ROS_ASSERT(joint_struct["parameters"].getType() == XmlRpcValue::TypeStruct);
                        LoadNanotecParametersFromStruct(parameters, joint_struct["parameters"]);
                    }

                    try {
                        joints_[i] = new aleph2_joint::NanotecJoint(
                            static_cast<int>(joint_struct["node_id"]),
                            joint_struct["busname"],
                            joint_struct["baudrate"],
                            Nanotec::OperationMode::VELOCITY,
                            static_cast<double>(joint_struct["scale"]),
                            parameters
                        );
                    } catch (const char* msg) {
                        ROS_ERROR_STREAM(joint_names_[i] << ": " << msg);
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Incorrect joint type: " << joint_struct["type"]);
                    ROS_ASSERT(false);
                }
            }, joint.second, i);
            ++i;
        }

        for( int i = 0; i < num_joints_; ++i )
        {
            joint_init_thread[i].join();
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
                aleph2_joint::JointType type = joints_[i]->getType();
                switch(type)
                {
                case aleph2_joint::JointType::RUBI_STEPPER:
                    joint_velocity_[i] = joints_[i]->getVelocity();
                    joint_position_[i] = joints_[i]->getPosition();
                    break;
                case aleph2_joint::JointType::NANOTEC:
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
                aleph2_joint::JointType type = joints_[i]->getType();
                switch(type)
                {
                case aleph2_joint::JointType::RUBI_STEPPER:
                    joints_[i]->setVelocity(joint_velocity_command_[i]);
                    break;
                case aleph2_joint::JointType::NANOTEC:
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