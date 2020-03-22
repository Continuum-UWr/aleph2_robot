#ifndef ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_JOINT_ADDON_H_
#define ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_JOINT_ADDON_H_

#include <string>

#include <ros/ros.h>

#include <aleph2_joint/joint.h>
#include <rubi_server/RubiUnsignedInt.h>

namespace aleph2_joint {
class JointAddon : public Joint {
 public:
  explicit JointAddon(Joint* joint) : joint_(joint) {}

  ~JointAddon() { delete joint_; }

  JointType getType() override { return joint_->getType(); }
  void setEffort(double effort) override { joint_->setEffort(effort); }
  void setVelocity(double velocity) override { joint_->setVelocity(velocity); }
  void setPosition(double position) override { joint_->setPosition(position); }
  double getEffort() override { return joint_->getEffort(); }
  double getVelocity() override { return joint_->getVelocity(); }
  double getPosition() override { return joint_->getPosition(); }

 private:
  Joint* joint_;
};

class RubiEncoderAddon : public JointAddon {
 public:
  RubiEncoderAddon(Joint* joint, const std::string& board_name,
                   const std::string& board_id,
                   const std::string& get_position_field, const double scale,
                   const double offset);
  double getPosition() override { return position_; }

 private:
  double position_, scale_, offset_;
  ros::NodeHandle nh_;
  ros::Subscriber pos_sub_;
  void positionCallback(const rubi_server::RubiUnsignedIntConstPtr& msg);
};
}  // namespace aleph2_joint

#endif  // ALEPH2_HARDWARE_INTERFACE_INCLUDE_ALEPH2_JOINT_ADDON_H_
