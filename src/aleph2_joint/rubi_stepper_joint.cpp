#include "ros/ros.h"
#include "rubi_server/RubiInt.h"

#include "aleph2_joint/joint.h"

namespace aleph2_joint
{

    RubiStepperJoint::RubiStepperJoint(std::string position_topic, 
                                       std::string velocity_topic, double scale)
        : scale_(scale)
    {
        vel_pub = nh.advertise<rubi_server::RubiInt>(velocity_topic, 10);
        pos_sub = nh.subscribe(position_topic, 10, &RubiStepperJoint::positionCallback, this);
    }

    JointType RubiStepperJoint::getType()
    {
        return JointType::RUBI_STEPPER;
    }

    void RubiStepperJoint::setVelocity(double velocity) 
    {
        velocity_ = velocity;
        rubi_server::RubiInt msg;
        msg.data.resize(1);
        msg.data[0] = static_cast<int32_t>(velocity * scale_);
        vel_pub.publish(msg);
    }

    void RubiStepperJoint::positionCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        position_ = static_cast<double>(msg->data[0]) / scale_;
    }

}