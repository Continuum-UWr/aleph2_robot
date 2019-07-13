#include "nanotec_driver/nanotec.h"
#include "aleph2_joint/joint.h"
 
namespace aleph2_joint
{
    NanotecJoint::NanotecJoint(const uint8_t node_id, const std::string& busname, 
                               const std::string& baudrate, const Nanotec::OperationMode op_mode,
                               const std::map<std::string, int64_t>& parameters)
    {
        if (!master_.start(busname, baudrate))
        {
            throw "Could not initialize can";
        }

        bool found_device = false;
        size_t device_index;

        while (!found_device)
        {
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
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }

        kaco::Device& device = master_.get_device(device_index);

        device.start();
        device.load_dictionary_from_library();

        nanotec_ = new Nanotec(device, op_mode);
        nanotec_->LoadParameters(parameters);
        nanotec_->SetPowerMode(Nanotec::PowerMode::ACTIVE);
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
        int32_t vel = static_cast<int32_t>(velocity);
        nanotec_->SetTarget(vel);
    }
    void NanotecJoint::setPosition(double position)
    {
        int32_t pos = static_cast<int32_t>(position);
        nanotec_->SetTarget(pos);
    }

    double NanotecJoint::getEffort()
    {
        return nanotec_->GetTorque();
    }

    double NanotecJoint::getVelocity() 
    {
        return nanotec_->GetVelocity();
    }

    double NanotecJoint::getPosition()
    {
        return nanotec_->GetPosition();
    }

}