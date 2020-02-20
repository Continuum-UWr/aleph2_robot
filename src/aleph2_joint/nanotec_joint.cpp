#include "nanotec_driver/nanotec.h"
#include "aleph2_joint/joint.h"
#include "kacanopen/core/sdo_error.h"
 
namespace aleph2_joint
{
    NanotecJoint::NanotecJoint(const kaco::Master& master,
                               const uint8_t node_id,
                               const std::string& nh_namespace,
                               const double scale, 
                               const std::map<std::string, int64_t>& parameters,
                               const Nanotec::OperationMode& op_mode)
        : scale_(scale),
          nh_(nh_namespace),
          reconfigure_server_(nh_),
          op_mode_(op_mode)
    {
        bool found_device = false;
        size_t device_index;

        while (!found_device)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            for (size_t i = 0; i < master.num_devices(); ++i)
            {
                kaco::Device& device = master.get_device(i);
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

        kaco::Device& device = master.get_device(device_index);

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
        setTarget(effort);
    }

    void NanotecJoint::setVelocity(double velocity)
    {
        setTarget(velocity * scale_);
    }
    void NanotecJoint::setPosition(double position)
    {
        setTarget(position * scale_);
    }

    void NanotecJoint::setTarget(double target)
    {
        int32_t msg = static_cast<int32_t>(target);
        try {
            nanotec_->SetTarget(msg);
        } catch(kaco::sdo_error& ex) {
            throw "Setting target failed!";
        }
    }

    double NanotecJoint::getEffort()
    {
        try {
            return static_cast<double>(nanotec_->GetTorque());
        } catch(kaco::sdo_error& ex) {
            throw "Getting effort failed!";
        }
    }

    double NanotecJoint::getVelocity() 
    {
        try {
            return static_cast<double>(nanotec_->GetVelocity()) / scale_;
        } catch(kaco::sdo_error& ex) {
            throw "Getting velocity failed!";
        }
    }

    double NanotecJoint::getPosition()
    {
        try {
            return static_cast<double>(nanotec_->GetPosition()) / scale_;
        } catch(kaco::sdo_error& ex) {
            throw "Getting velocity failed!";
        }
    }

}