#ifndef ALEPH2CPP_JOINT_H
#define ALEPH2CPP_JOINT_H

#include <string>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

namespace aleph2cpp
{

    class RubiStepperJoint
    {
    public:
        RubiStepperJoint(std::string position_topic, std::string velocity_topic)
            : position_(0)
        {
            vel_pub = nh.advertise<std_msgs::Int32>(velocity_topic, 10);
            pos_sub = nh.subscribe(position_topic, 10, &RubiStepperJoint::positionCallback, this);
        }
        void setVelocity(float velocity) 
        {
            std_msgs::Int32 msg;
            msg.data = static_cast<int32_t>(velocity);
            vel_pub.publish(msg);
        }
        float getPosition() { return position_; }
    private:
        float position_;
        ros::NodeHandle nh;
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
        void positionCallback(const std_msgs::Int32::ConstPtr& msg)
        {
            position_ = static_cast<float>(msg->data);
        }
    };

}

#endif