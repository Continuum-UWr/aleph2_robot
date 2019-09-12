#include "ros/ros.h"
#include "rubi_server/RubiInt.h"
#include "rubi_server/RubiBool.h"
#include "std_srvs/Trigger.h"

#include "aleph2_joint/joint.h"

namespace aleph2_joint
{

    RubiJoint::RubiJoint(const std::string& board_name,
                         const std::string& board_id, 
                         const std::string& get_effort_field,
                         const std::string& get_velocity_field,
                         const std::string& get_position_field,
                         const std::string& set_effort_field,
                         const std::string& set_velocity_field,
                         const std::string& set_position_field,
                         const std::string& home_field, 
                         const std::string& nh_namespace,
                         double scale)
        : scale_(scale),
          nh_(nh_namespace),
          home_(false),
          effort_(0.0),
          velocity_(0.0),
          position_(0.0)
    {

        std::string base_topic = "/rubi/boards/" + board_name + '/';
        if (!board_id.empty())
            base_topic += board_id + '/';

        if (!get_effort_field.empty())  
            eff_sub_ = nh_.subscribe(
                base_topic + "fields_from_board/" + get_effort_field, 10,
                &RubiJoint::effortCallback, this
            );

        if (!get_velocity_field.empty())  
            vel_sub_ = nh_.subscribe(
                base_topic + "fields_from_board/" + get_velocity_field, 10,
                &RubiJoint::velocityCallback, this
            );

        if (!get_position_field.empty())
            pos_sub_ = nh_.subscribe(
                base_topic + "fields_from_board/" + get_position_field, 10,
                &RubiJoint::positionCallback, this
            );

        if (!set_effort_field.empty())
            eff_pub_ = nh_.advertise<rubi_server::RubiInt>(
                base_topic + "fields_to_board/" + set_effort_field, 10);
            
        if (!set_velocity_field.empty())
            vel_pub_ = nh_.advertise<rubi_server::RubiInt>(
                base_topic + "fields_to_board/" + set_velocity_field, 10);

        if (!set_position_field.empty())
            pos_pub_ = nh_.advertise<rubi_server::RubiInt>(
                base_topic + "fields_to_board/" + set_position_field, 10);

        if (!home_field.empty())
        {
            home_pub_ = nh_.advertise<rubi_server::RubiBool>(
                base_topic + "fields_to_board/" + home_field, 1);

            home_sub_ = nh_.subscribe(
                base_topic + "fields_from_board/" + home_field, 1,
                &RubiJoint::homeCallback, this
            );

            home_service_ = nh_.advertiseService("home", &RubiJoint::homeFunction, this);

        }
    }

    JointType RubiJoint::getType()
    {
        return JointType::RUBI;
    }

    void RubiJoint::setEffort(double effort) 
    {
        if (!eff_pub_.getTopic().empty()) 
        {
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(effort);
            eff_pub_.publish(msg);
        }
    }

    void RubiJoint::setVelocity(double velocity) 
    {
        if (!vel_pub_.getTopic().empty()) 
        {
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(velocity * scale_);
            vel_pub_.publish(msg);
        }
    }

    void RubiJoint::setPosition(double position) 
    {
        if (!pos_pub_.getTopic().empty()) 
        {
            rubi_server::RubiInt msg;
            msg.data.resize(1);
            msg.data[0] = static_cast<int32_t>(position * scale_);
            pos_pub_.publish(msg);
        }
    }

    void RubiJoint::effortCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        effort_ = static_cast<double>(msg->data[0]);
    }

    void RubiJoint::velocityCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        velocity_ = static_cast<double>(msg->data[0]) / scale_;
    }

    void RubiJoint::positionCallback(const rubi_server::RubiIntConstPtr& msg)
    {
        position_ = static_cast<double>(msg->data[0]) / scale_;
    }

    void RubiJoint::homeCallback(const rubi_server::RubiBoolConstPtr& msg)
    {
        home_ = msg->data[0];
    }

    bool RubiJoint::homeFunction(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res)
    {
        if (home_) 
        {
            res.message = "The joint is already homing!";
            res.success = false;
            return true;
        }
        else 
        {
            rubi_server::RubiBool msg;
            msg.data.push_back(true);
            home_pub_.publish(msg);

            ros::Time begin = ros::Time::now();
            ros::Rate rate(10);
            
            while (ros::ok() && !home_)
            {
                if ((ros::Time::now() - begin).sec > 5)
                {
                    res.message = "Could not start homing!";
                    res.success = false;
                    return true;
                }
                rate.sleep();
            }

            if (!ros::ok())
                return false;
            
            begin = ros::Time::now();
            rate.reset();
            while (ros::ok() && home_)
            {
                if ((ros::Time::now() - begin).sec > 10)
                {
                    res.message = "Homing took too long!";
                    res.success = false;
                    return true;
                }
                rate.sleep();
            }

            if (!ros::ok())
                return false;
        }
        res.message = "Joint succesfully homed!";
        res.success = true;
        return true;
    }

}