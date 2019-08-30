#include <math.h>

#include "ros/ros.h"
#include "rubi_server/RubiUnsignedInt.h"

#include "aleph2_joint/addon.h"

namespace aleph2_joint
{

    RubiEncoderAddon::RubiEncoderAddon(Joint* joint, std::string position_topic, double offset)
        : JointAddon(joint),
          offset_(offset)
    {
        pos_sub_ = nh_.subscribe(position_topic, 10, &RubiEncoderAddon::positionCallback, this);
    }

    void RubiEncoderAddon::positionCallback(const rubi_server::RubiUnsignedIntConstPtr& msg)
    {
        position_ = static_cast<double>(msg->data[0]) * (2 * M_PI / 4096.0) + offset_;
    }
}