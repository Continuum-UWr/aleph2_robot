#ifndef ALEPH2CPP_JOINT_H
#define ALEPH2CPP_JOINT_H

#include <string>

#include "ros/ros.h"
#include "rubi_server/RubiInt.h"
#include "nanotec_driver/nanotec.h"

namespace aleph2cpp
{
    enum class JointType 
    {
        RUBI_STEPPER,
        NANOTEC
    };

    class Joint
    {
    public:
        virtual JointType getType() = 0;
        virtual void setEffort(double effort) { throw "setEffort is not implemented"; }
        virtual void setVelocity(double velocity) { throw "setVelocity is not implemented"; }
        virtual void setPosition(double position) { throw "setPosition is not implemented"; }
        virtual double getEffort() { throw "getEffort is not implemented"; }
        virtual double getVelocity() { throw "getVelocity is not implemented"; }
        virtual double getPosition() { throw "getPosition is not implemented"; }
    };

    class RubiStepperJoint : public Joint
    {
    public:
        RubiStepperJoint(std::string position_topic, 
                         std::string velocity_topic, 
                         float scale)
            : position_(0),
              velocity_(0),
              scale_(scale)
        {
            vel_pub = nh.advertise<rubi_server::RubiInt>(velocity_topic, 10);
            pos_sub = nh.subscribe(position_topic, 10, &RubiStepperJoint::positionCallback, this);
        }

        JointType getType()
        {
            return JointType::RUBI_STEPPER;
        }

        void setVelocity(double velocity) 
        {
            velocity_ = velocity;
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(velocity * scale_);
            vel_pub.publish(msg);
        }

        double getVelocity() { return velocity_; }
        double getPosition() { return position_; }

    private:
        double velocity_, position_, scale_;
        ros::NodeHandle nh;
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
        void positionCallback(const rubi_server::RubiIntConstPtr& msg)
        {
            position_ = static_cast<double>(msg->data[0]) / scale_;
        }
    };


    class NanotecJoint: public Joint
    {
    public:
        NanotecJoint(const uint8_t node_id, 
                     const std::string& busname, 
                     const std::string& baudrate,
                     Nanotec::OperationMode op_mode)
        {
            kaco::Master master;
            if (!master.start(busname, baudrate))
            {
                throw "Could not initialize can";
            }

            bool found_device = false;
            size_t device_index;

            while (!found_device)
            {
                for (size_t i = 0; i < master.num_devices(); ++i)
                {
                    kaco::Device &device = master.get_device(i);
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

            kaco::Device &device = master.get_device(device_index);

            device.start();
            device.load_dictionary_from_library();

            nanotec_ = new Nanotec(device, op_mode);
        }

        JointType getType()
        {
            return JointType::NANOTEC;
        }

        //void setEffort(double effort);
        void setVelocity(double velocity)
        {
            int32_t vel = static_cast<int32_t>(velocity);
            nanotec_->SetTarget(vel);
        }
        void setPosition(double position)
        {
            int32_t pos = static_cast<int32_t>(position);
            nanotec_->SetTarget(pos);
        }

        //double getEffort() 
        double getVelocity() 
        {
            return nanotec_->GetVelocity();
        }
        double getPosition()
        {
            return nanotec_->GetPosition();
        }

    private:
        Nanotec* nanotec_;
    };

}

#endif