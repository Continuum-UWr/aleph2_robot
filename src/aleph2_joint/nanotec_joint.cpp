#include "nanotec_driver/nanotec.h"
#include "aleph2_joint/joint.h"
 
namespace aleph2_joint
{
    NanotecJoint::NanotecJoint(const uint8_t node_id, const std::string& busname, 
                               const std::string& baudrate, const std::string& nh_namespace,
                               double scale, const std::map<std::string, int64_t>& parameters,
                               const Nanotec::OperationMode& op_mode)
        : scale_(scale),
          nh_(nh_namespace),
          reconfigure_server_(nh_),
          op_mode_(op_mode)
    {
        if (!master_.start(busname, baudrate))
        {
            throw "Could not initialize can";
        }

        bool found_device = false;
        size_t device_index;

        while (!found_device)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            for (size_t i = 0; i < master_.num_devices(); ++i)
            {
                kaco::Device &device = master_.get_device(i);
                if (device.get_node_id() == node_id)
                {
                    found_device = true;
                    device_index = i;
                    break;
                }
            }

            ROS_WARN_STREAM("Device with ID " << (unsigned)node_id
                            << " has not been found yet. Will keep retrying.");
        }

        kaco::Device& device = master_.get_device(device_index);

        device.start();
        device.load_dictionary_from_library();

        nanotec_ = new Nanotec(device, op_mode_);
        nanotec_->LoadParameters(parameters);

        call_type_ = boost::bind(&NanotecJoint::configCallback, this, _1, _2);
        reconfigure_server_.setCallback(call_type_);
    }

    void NanotecJoint::configCallback(NanotecConfig& config, uint32_t level)
    {
        if (config.power) 
        {
            if (power_mode_ != Nanotec::PowerMode::ACTIVE)
            {
                power_mode_ = Nanotec::PowerMode::ACTIVE;
                nanotec_->SetPowerMode(power_mode_);
            }
            if (config.brake)
            {
                active_braking_ = true;
                if (op_mode_ == Nanotec::OperationMode::VELOCITY)
                    nanotec_->SetTarget(0);
            }
            else
            {
                active_braking_ = false;
            }
            
        }
        else if (config.brake)
        {
            if (power_mode_ != Nanotec::PowerMode::PASSIVE_BRAKE)
            {
                power_mode_ = Nanotec::PowerMode::PASSIVE_BRAKE;
                nanotec_->SetPowerMode(power_mode_);
            }
            active_braking_ = false;
        }
        else
        {
            if (power_mode_ != Nanotec::PowerMode::OFF)
            {
                power_mode_ = Nanotec::PowerMode::OFF;
                nanotec_->SetPowerMode(power_mode_);
            }
        }
        
    }

    JointType NanotecJoint::getType()
    {
        return JointType::NANOTEC;
    }

    void NanotecJoint::setEffort(double effort)
    {
        int32_t eff = static_cast<int32_t>(effort);
        nanotec_->SetTarget(eff);
    }

    void NanotecJoint::setVelocity(double velocity)
    {
        if (!active_braking_)
        {
            int32_t vel = static_cast<int32_t>(velocity * scale_);
            nanotec_->SetTarget(vel);
        }
    }
    void NanotecJoint::setPosition(double position)
    {
        int32_t pos = static_cast<int32_t>(position * scale_);
        nanotec_->SetTarget(pos);
    }

    double NanotecJoint::getEffort()
    {
        return static_cast<double>(nanotec_->GetTorque());
    }

    double NanotecJoint::getVelocity() 
    {
        return static_cast<double>(nanotec_->GetVelocity()) / scale_;
    }

    double NanotecJoint::getPosition()
    {
        return static_cast<double>(nanotec_->GetPosition()) / scale_;
    }

}