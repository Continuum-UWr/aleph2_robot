#ifndef ALEPH2_JOINT_ADDON_H
#define ALEPH2_JOINT_ADDON_H

#include <string>

#include "ros/ros.h"
#include "rubi_server/RubiUnsignedInt.h"
#include "aleph2_joint/joint.h"


namespace aleph2_joint
{
    class JointAddon : public Joint
    {
    public:
        JointAddon(Joint* joint) 
            : joint_(joint) { }

        ~JointAddon() { delete joint_; }
        
        virtual JointType getType() override { return joint_->getType(); }
        virtual void setEffort(double effort) override { joint_->setEffort(effort); }
        virtual void setVelocity(double velocity) override { joint_->setVelocity(velocity); }
        virtual void setPosition(double position) override { joint_->setPosition(position); }
        virtual double getEffort() override { return joint_->getEffort(); }
        virtual double getVelocity() override { return joint_->getVelocity(); }
        virtual double getPosition() override { return joint_->getPosition(); }
    private:
        Joint* joint_;
    };


    class RubiEncoderAddon : public JointAddon
    {
    public:
        RubiEncoderAddon(Joint* joint, std::string position_topic, double offset);
        virtual double getPosition() override { return position_; }
    private:
        double position_, offset_;
        ros::NodeHandle nh_;
        ros::Subscriber pos_sub_;
        void positionCallback(const rubi_server::RubiUnsignedIntConstPtr& msg);
    };
}

#endif