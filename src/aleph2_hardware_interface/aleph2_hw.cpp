#include <aleph2_hardware_interface/aleph2_hw.h>
#include <aleph2_hardware_interface/utils.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <nanotec_driver/nanotec.h>

using namespace hardware_interface;
using namespace joint_limits_interface;
using XmlRpc::XmlRpcValue;

namespace aleph2_hardware_interface
{
    Aleph2HW::Aleph2HW() {}
    Aleph2HW::~Aleph2HW() {}

    void Aleph2HW::init(ros::NodeHandle& robot_hw_nh) {
        XmlRpcValue hardware, can_devices, nanotec_presets;
        robot_hw_nh.getParam("joints", hardware);
        robot_hw_nh.getParam("can_devices", can_devices);
        robot_hw_nh.getParam("nanotec_presets", nanotec_presets);

        ROS_ASSERT(hardware.getType() == XmlRpcValue::TypeStruct);
        ROS_ASSERT(can_devices.getType() == XmlRpcValue::TypeStruct || !can_devices.valid());
        ROS_ASSERT(nanotec_presets.getType() == XmlRpcValue::TypeStruct || !nanotec_presets.valid());

        num_joints_ = hardware.size();

        // Resize vectors
        joint_names_.resize(num_joints_);
        joint_modes_.resize(num_joints_);
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);
        joints_.resize(num_joints_);


        for( auto const& device : can_devices )
        {
            std::string device_name = device.first;
            auto device_struct = device.second;

            ROS_ASSERT(device_struct.hasMember("busname"));
            ROS_ASSERT(device_struct["busname"].getType() == XmlRpcValue::TypeString);
            ROS_ASSERT(device_struct.hasMember("baudrate"));
            ROS_ASSERT(device_struct["baudrate"].getType() == XmlRpcValue::TypeString);

            std::string busname = device_struct["busname"];
            std::string baudrate = device_struct["baudrate"];

            kaco::Master* master = new kaco::Master();
            if( !master->start(busname, baudrate) )
            {
                ROS_ERROR_STREAM("failed to start can master for device " << device_name 
                    << " (busname: " << busname << ", baudrate: " << baudrate << ")");
                ROS_BREAK();
            }

            can_masters_[device_name] = master;
        }

        int i = 0;
        for( auto const& joint : hardware )
        {
            joint_names_[i] = joint.first;
            auto joint_struct = joint.second;
            joint_modes_[i] = JointMode::NONE;
            
            ROS_ASSERT(joint_struct.getType() == XmlRpcValue::TypeStruct);

            ROS_ASSERT(joint_struct.hasMember("type"));
            ROS_ASSERT(joint_struct["type"].getType() == XmlRpcValue::TypeString);

            if( joint_struct["type"] == "rubi" )
            {
                // Create joint handles
                JointStateHandle joint_state_handle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
                JointHandle joint_effort_handle(joint_state_handle, &joint_effort_command_[i]);
                JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i]);
                JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i]);

                // Get joint limits
                JointLimits joint_limits;
                SoftJointLimits joint_soft_limits;
                bool has_limits = getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
                bool has_soft_limits = getSoftJointLimits(joint_names_[i], robot_hw_nh, joint_soft_limits);

                // Register handles
                joint_state_interface_.registerHandle(joint_state_handle);
                effort_joint_interface_.registerHandle(joint_effort_handle);
                velocity_joint_interface_.registerHandle(joint_velocity_handle);
                position_joint_interface_.registerHandle(joint_position_handle);

                if (has_limits) 
                {
                    registerLimitsHandles(joint_effort_handle, joint_velocity_handle, joint_position_handle,
                        joint_limits, has_soft_limits, joint_soft_limits);
                }

                ROS_ASSERT(joint_struct.hasMember("board_name"));
                ROS_ASSERT(joint_struct["board_name"].getType() == XmlRpcValue::TypeString);
                ROS_ASSERT(joint_struct.hasMember("scale"));
                ROS_ASSERT(joint_struct["scale"].getType() == XmlRpcValue::TypeDouble);
                
                // optional parameters
                std::string board_id, home_field,
                            get_effort_field, get_velocity_field, get_position_field,
                            set_effort_field, set_velocity_field, set_position_field;

                board_id = LoadOptionalRubiFieldFromStruct("board_id", joint_struct);
                home_field = LoadOptionalRubiFieldFromStruct("home_field", joint_struct);
                get_effort_field = LoadOptionalRubiFieldFromStruct("get_effort_field", joint_struct);
                get_velocity_field = LoadOptionalRubiFieldFromStruct("get_velocity_field", joint_struct);
                get_position_field = LoadOptionalRubiFieldFromStruct("get_position_field", joint_struct);
                set_effort_field = LoadOptionalRubiFieldFromStruct("set_effort_field", joint_struct);
                set_velocity_field = LoadOptionalRubiFieldFromStruct("set_velocity_field", joint_struct);
                set_position_field = LoadOptionalRubiFieldFromStruct("set_position_field", joint_struct);

                joints_[i] = new aleph2_joint::RubiJoint(
                    joint_struct["board_name"],
                    board_id,
                    get_effort_field,
                    get_velocity_field,
                    get_position_field,
                    set_effort_field,
                    set_velocity_field,
                    set_position_field,
                    home_field,
                    robot_hw_nh.getNamespace() + "/joints/" + joint_names_[i],
                    static_cast<double>(joint_struct["scale"])
                );

            }
            else if( joint_struct["type"] == "nanotec" )
            {
                // Create joint handles
                JointStateHandle joint_state_handle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
                JointHandle joint_effort_handle(joint_state_handle, &joint_effort_command_[i]);
                JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i]);
                JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i]);

                // Create joint limits handles
                JointLimits joint_limits;
                SoftJointLimits joint_soft_limits;
                bool has_limits = getJointLimits(joint_names_[i], robot_hw_nh, joint_limits);
                bool has_soft_limits = getSoftJointLimits(joint_names_[i], robot_hw_nh, joint_soft_limits);

                // Register handles
                joint_state_interface_.registerHandle(joint_state_handle);
                effort_joint_interface_.registerHandle(joint_effort_handle);
                velocity_joint_interface_.registerHandle(joint_velocity_handle);
                position_joint_interface_.registerHandle(joint_position_handle);

                if (has_limits) 
                {
                    registerLimitsHandles(joint_effort_handle, joint_velocity_handle, joint_position_handle,
                        joint_limits, has_soft_limits, joint_soft_limits);
                }

                ROS_ASSERT(joint_struct.hasMember("can_device"));
                ROS_ASSERT(joint_struct["can_device"].getType() == XmlRpcValue::TypeString);
                ROS_ASSERT(joint_struct.hasMember("node_id"));
                ROS_ASSERT(joint_struct["node_id"].getType() == XmlRpcValue::TypeInt);
                ROS_ASSERT(joint_struct.hasMember("scale"));
                ROS_ASSERT(joint_struct["scale"].getType() == XmlRpcValue::TypeDouble);

                kaco::Master* master;

                try
                {
                    master = can_masters_.at(joint_struct["can_device"]);
                }
                catch (const std::out_of_range& ex)
                {
                    ROS_ERROR_STREAM("No can master for device " << joint_struct["can_device"] << " found");
                    ROS_BREAK();
                }

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

                joints_[i] = new aleph2_joint::NanotecJoint(
                    *master,
                    static_cast<int>(joint_struct["node_id"]),
                    robot_hw_nh.getNamespace() + "/joints/" + joint_names_[i],
                    static_cast<double>(joint_struct["scale"]),
                    parameters
                );
            }
            else
            {
                ROS_ERROR_STREAM("Incorrect joint type: " << joint_struct["type"]);
                ROS_BREAK();
            }

            if (joint_struct.hasMember("has_rubi_encoder"))
            {
                ROS_ASSERT(joint_struct["has_rubi_encoder"].getType() == XmlRpcValue::TypeBoolean);
                if (joint_struct["has_rubi_encoder"])
                {
                    ROS_ASSERT(joint_struct.hasMember("encoder_position_topic"));
                    ROS_ASSERT(joint_struct["encoder_position_topic"].getType() == XmlRpcValue::TypeString);
                    ROS_ASSERT(joint_struct.hasMember("encoder_angle_offset"));
                    ROS_ASSERT(joint_struct["encoder_angle_offset"].getType() == XmlRpcValue::TypeDouble);
                    joints_[i] = new aleph2_joint::RubiEncoderAddon(
                        joints_[i],
                        joint_struct["encoder_position_topic"],
                        joint_struct["encoder_angle_offset"]
                    );
                }
            }

            ++i;
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&effort_joint_saturation_interface_);
        registerInterface(&effort_joint_soft_limits_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&velocity_joint_saturation_interface_);
        registerInterface(&velocity_joint_soft_limits_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&position_joint_saturation_interface_);
        registerInterface(&position_joint_soft_limits_interface_);
    }

    void Aleph2HW::registerLimitsHandles(JointHandle& joint_effort_handle, JointHandle& joint_velocity_handle,
            JointHandle& joint_position_handle, JointLimits& joint_limits, bool has_soft_limits, SoftJointLimits& joint_soft_limits)
    {
        if (!has_soft_limits)
        {
            if (joint_limits.has_effort_limits && joint_limits.has_velocity_limits)
            {
                EffortJointSaturationHandle joint_effort_saturation_handle(
                    joint_effort_handle, joint_limits);
                effort_joint_saturation_interface_.registerHandle(joint_effort_saturation_handle);
            }
            if (joint_limits.has_velocity_limits)
            {
                VelocityJointSaturationHandle joint_velocity_saturation_handle(
                    joint_velocity_handle, joint_limits);
                velocity_joint_saturation_interface_.registerHandle(joint_velocity_saturation_handle);
            }
            if (joint_limits.has_position_limits)
            {
                PositionJointSaturationHandle joint_position_saturation_handle(
                    joint_position_handle, joint_limits);
                position_joint_saturation_interface_.registerHandle(joint_position_saturation_handle);
            }
        }
        else
        {
            if (joint_limits.has_effort_limits && joint_limits.has_velocity_limits)
            {
                EffortJointSoftLimitsHandle joint_effort_soft_limits_handle(
                    joint_effort_handle, joint_limits, joint_soft_limits);
                effort_joint_soft_limits_interface_.registerHandle(joint_effort_soft_limits_handle);
            }

            VelocityJointSoftLimitsHandle joint_velocity_soft_limits_handle(
                joint_velocity_handle, joint_limits, joint_soft_limits);
            velocity_joint_soft_limits_interface_.registerHandle(joint_velocity_soft_limits_handle);

            if (joint_limits.has_velocity_limits)
            {
                PositionJointSoftLimitsHandle joint_position_soft_limits_handle(
                    joint_position_handle, joint_limits, joint_soft_limits);
                position_joint_soft_limits_interface_.registerHandle(joint_position_soft_limits_handle);
            }
        }
        
    }

    void Aleph2HW::doSwitch(const std::list<ControllerInfo>& start_controllers,
                                           const std::list<ControllerInfo>& stop_controllers)
    {
        for (const ControllerInfo& con : start_controllers)
        {
            for (const InterfaceResources& res : con.claimed_resources)
            {
                JointMode jm = JointMode::NONE;
                if (res.hardware_interface == "hardware_interface::EffortJointInterface")
                    jm = JointMode::EFFORT;
                else if (res.hardware_interface == "hardware_interface::VelocityJointInterface")
                    jm = JointMode::VELOCITY;
                else if (res.hardware_interface == "hardware_interface::PositionJointInterface")
                    jm = JointMode::POSITION;

                if (jm != JointMode::NONE)
                {
                    ROS_ASSERT(res.resources.size() > 0);
                    const std::string& name = *res.resources.begin();
                    for (int i = 0; i < num_joints_; ++i)
                    {
                        if (joint_names_[i] == name) 
                        {
                            joint_modes_[i] = jm;
                            break;
                        }
                    }
                }
            }
        }
    }

    void Aleph2HW::read() 
    {
        for( int i = 0; i < num_joints_; ++i )
        {
            try {
                aleph2_joint::JointType type = joints_[i]->getType();
                switch(type)
                {
                case aleph2_joint::JointType::RUBI:
                    joint_effort_[i] = joints_[i]->getEffort();
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

    void Aleph2HW::write(ros::Duration elapsed_time) {
        effort_joint_saturation_interface_.enforceLimits(elapsed_time);
        effort_joint_soft_limits_interface_.enforceLimits(elapsed_time);
        velocity_joint_saturation_interface_.enforceLimits(elapsed_time);
        velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);
        position_joint_saturation_interface_.enforceLimits(elapsed_time);
        position_joint_soft_limits_interface_.enforceLimits(elapsed_time);

        for( int i = 0; i < num_joints_; ++i )
        {
            try {
                if (joints_[i]->getType() == aleph2_joint::JointType::NANOTEC)
                {
                    joints_[i]->setVelocity(joint_velocity_command_[i]);
                }
                else
                {
                    switch(joint_modes_[i])
                    {
                    case JointMode::EFFORT:
                        joints_[i]->setEffort(joint_effort_command_[i]);
                        break;
                    case JointMode::VELOCITY:
                        joints_[i]->setVelocity(joint_velocity_command_[i]);
                        break;
                    case JointMode::POSITION:
                        joints_[i]->setPosition(joint_position_command_[i]);
                        break;
                    }
                }
            } catch (const char* msg) {
                ROS_ERROR_STREAM(msg);
            }
            
        }
    }
}