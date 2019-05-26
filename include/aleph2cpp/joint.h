#ifndef ALEPH2CPP_JOINT_H
#define ALEPH2CPP_JOINT_H

#include <string>

#include "ros/ros.h"
#include "rubi_server/RubiInt.h"

namespace aleph2cpp
{

    class RubiStepperJoint
    {
    public:
        RubiStepperJoint(std::string position_topic, std::string velocity_topic, float scale)
            : position_(0),
              scale_(scale)
        {
            vel_pub = nh.advertise<rubi_server::RubiInt>(velocity_topic, 10);
            pos_sub = nh.subscribe(position_topic, 10, &RubiStepperJoint::positionCallback, this);
        }
        void setVelocity(double velocity) 
        {
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(velocity * scale_);
            vel_pub.publish(msg);
        }
        float getPosition() { return position_; }
    private:
        double position_, scale_;
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