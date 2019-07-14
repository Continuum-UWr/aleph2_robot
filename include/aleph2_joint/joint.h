#ifndef ALEPH2CPP_JOINT_H
#define ALEPH2CPP_JOINT_H

#include <string>
#include <map>

#include "ros/ros.h"
#include "rubi_server/RubiInt.h"
#include "nanotec_driver/nanotec.h"

namespace aleph2_joint
{
    enum class JointType 
    {
        RUBI_STEPPER,
        NANOTEC
    };

    class Joint
    {
    public:
        virtual JointType getType() = 0;
        virtual void setEffort(double effort) { throw "setEffort is not implemented"; }
        virtual void setVelocity(double velocity) { throw "setVelocity is not implemented"; }
        virtual void setPosition(double position) { throw "setPosition is not implemented"; }
        virtual double getEffort() { throw "getEffort is not implemented"; }
        virtual double getVelocity() { throw "getVelocity is not implemented"; }
        virtual double getPosition() { throw "getPosition is not implemented"; }
    };

    class RubiStepperJoint : public Joint
    {
    public:
        RubiStepperJoint(std::string position_topic, std::string velocity_topic, double scale);

        JointType getType();
        
        void setVelocity(double velocity); 

        double getVelocity() { return velocity_; }
        double getPosition() { return position_; }

    private:
        double velocity_, position_, scale_;
        ros::NodeHandle nh;
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
        void positionCallback(const rubi_server::RubiIntConstPtr& msg);
    };


    class NanotecJoint: public Joint
    {
    public:
        NanotecJoint(const uint8_t node_id, const std::string& busname, 
                     const std::string& baudrate, const Nanotec::OperationMode op_mode,
                     double scale, const std::map<std::string, int64_t>& parameters);

        JointType getType();

        void setEffort(double effort);
        void setVelocity(double velocity);
        void setPosition(double position);

        double getEffort();
        double getVelocity(); 
        double getPosition();

    private:
        kaco::Master master_;
        Nanotec* nanotec_;
        double scale_;
    };

}

#endif