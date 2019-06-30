#ifndef ALEPH2CPP_JOINT_H
#define ALEPH2CPP_JOINT_H

#include <string>

#include "ros/ros.h"
#include "rubi_server/RubiInt.h"

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
        RubiStepperJoint(std::string position_topic, std::string velocity_topic, float scale)
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

}

#endif