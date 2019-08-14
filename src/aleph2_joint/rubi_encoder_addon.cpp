#include "ros/ros.h"
#include "rubi_server/RubiInt.h"

#include "aleph2_joint/addon.h"

namespace aleph2_joint
{

    RubiEncoderAddon::RubiEncoderAddon(Joint* joint, std::string position_topic)
        : JointAddon(joint)
    {
        pos_sub_ = nh_.subscribe(position_topic, 10, &RubiEncoderAddon::positionCallback, this);
    }

    void RubiEncoderAddon::positionCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        position_ = static_cast<double>(msg->data[0]);
    }
}