#include "ros/ros.h"
#include "rubi_server/RubiInt.h"
#include "rubi_server/RubiBool.h"
#include "std_srvs/Trigger.h"

#include "aleph2_joint/joint.h"

namespace aleph2_joint
{

    RubiStepperJoint::RubiStepperJoint(const std::string& board_name, const std::string& board_id, 
                            const std::string& position_field, const std::string& velocity_field,
                            const std::string& home_field, const std::string& nh_namespace,
                            double scale)
        : scale_(scale),
          nh_(nh_namespace)
    {
        std::string base_topic = "/rubi/boards/" + board_name + '/';
        if (!board_id.empty())
            base_topic += board_id + '/';

        vel_pub_ = nh_.advertise<rubi_server::RubiInt>(
            base_topic + "fields_to_board/" + velocity_field, 10);

        pos_sub_ = nh_.subscribe(
            base_topic + "fields_from_board/" + position_field, 10,
            &RubiStepperJoint::positionCallback, this
        );

        if (!home_field.empty())
        {
            home_pub_ = nh_.advertise<rubi_server::RubiBool>(
                base_topic + "fields_to_board/" + home_field, 1);

            home_sub_ = nh_.subscribe(
                base_topic + "fields_from_board/" + home_field, 1,
                &RubiStepperJoint::homeCallback, this
            );

            home_service_ = nh_.advertiseService("home", &RubiStepperJoint::homeFunction, this);

        }
    }

    JointType RubiStepperJoint::getType()
    {
        return JointType::RUBI_STEPPER;
    }

    void RubiStepperJoint::setVelocity(double velocity) 
    {
        if (!home_) {
            velocity_ = velocity;
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(velocity * scale_);
            vel_pub_.publish(msg);
        }
    }

    void RubiStepperJoint::positionCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        position_ = static_cast<double>(msg->data[0]) / scale_;
    }

    void RubiStepperJoint::homeCallback(const rubi_server::RubiBoolConstPtr& msg)
    {
        home_ = msg->data[0];
    }

    bool RubiStepperJoint::homeFunction(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res)
    {
        res.message = "TODO";
        res.success = false;
        return true;
    }

}