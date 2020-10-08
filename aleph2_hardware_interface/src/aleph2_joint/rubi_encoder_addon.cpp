#include <math.h>

#include "ros/ros.h"
#include "rubi_server/RubiUnsignedInt.h"

#include "aleph2_joint/addon.h"

namespace aleph2_joint {

RubiEncoderAddon::RubiEncoderAddon(Joint* joint, const std::string& board_name,
                                   const std::string& board_id,
                                   const std::string& get_position_field,
                                   const double scale, const double offset)
    : JointAddon(joint), scale_(scale), offset_(offset) {
  std::string base_topic = "/rubi/boards/" + board_name + '/';
  if (!board_id.empty()) base_topic += board_id + '/';

  pos_sub_ =
      nh_.subscribe(base_topic + "fields_from_board/" + get_position_field, 10,
                    &RubiEncoderAddon::positionCallback, this);
}

void RubiEncoderAddon::positionCallback(
    const rubi_server::RubiUnsignedIntConstPtr& msg) {
  position_ = static_cast<double>(msg->data[0]) / scale_ + offset_;
}
}  // namespace aleph2_joint
