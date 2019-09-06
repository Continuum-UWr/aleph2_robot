#ifndef ALEPH2_JOINT_JOINT_H
#define ALEPH2_JOINT_JOINT_H

#include <string>
#include <map>

#include "ros/ros.h"
#include "rubi_server/RubiInt.h"
#include "rubi_server/RubiBool.h"
#include "std_srvs/Trigger.h"
#include "nanotec_driver/nanotec.h"

#include "dynamic_reconfigure/server.h"
#include "aleph2_hardware_interface/NanotecConfig.h"


namespace aleph2_joint
{
    enum class JointType 
    {
        RUBI,
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


    class RubiJoint : public Joint
    {
    public:
        RubiJoint(const std::string& board_name,
                  const std::string& board_id, 
                  const std::string& get_effort_field,
                  const std::string& get_velocity_field,
                  const std::string& get_position_field,
                  const std::string& set_effort_field,
                  const std::string& set_velocity_field,
                  const std::string& set_position_field,
                  const std::string& home_field, 
                  const std::string& nh_namespace,
                  double scale);

        JointType getType();
        
        void setEffort(double effort);
        void setVelocity(double velocity);
        void setPosition(double position);

        double getEffort() { return effort_; }
        double getVelocity() { return velocity_; }
        double getPosition() { return position_; }

    private:
        double effort_, velocity_, position_;
        double scale_;
        ros::NodeHandle nh_;
        ros::Subscriber eff_sub_, vel_sub_, pos_sub_, home_sub_;
        ros::Publisher eff_pub_, vel_pub_, pos_pub_, home_pub_;
        ros::ServiceServer home_service_;

        void effortCallback(const rubi_server::RubiIntConstPtr& msg);
        void velocityCallback(const rubi_server::RubiIntConstPtr& msg);
        void positionCallback(const rubi_server::RubiIntConstPtr& msg);

        bool home_;
        void homeCallback(const rubi_server::RubiBoolConstPtr& msg);
        bool homeFunction(std_srvs::Trigger::Request& req,
                          std_srvs::Trigger::Response& res);
    };


    class NanotecJoint: public Joint
    {
    public:
        NanotecJoint(const uint8_t node_id, 
                     const std::string& busname, 
                     const std::string& baudrate, 
                     const std::string& nh_namespace,
                     double scale, 
                     const std::map<std::string, int64_t>& parameters,
                     const Nanotec::OperationMode& op_mode = Nanotec::OperationMode::VELOCITY);

        JointType getType();

        void setEffort(double effort);
        void setVelocity(double velocity);
        void setPosition(double position);

        double getEffort();
        double getVelocity(); 
        double getPosition();

    private:
        void configCallback(NanotecConfig& config, uint32_t level);

        ros::NodeHandle nh_;
        kaco::Master master_;
        Nanotec* nanotec_;
        dynamic_reconfigure::Server<NanotecConfig> reconfigure_server_;
        dynamic_reconfigure::Server<NanotecConfig>::CallbackType call_type_; 
        Nanotec::OperationMode op_mode_;
        Nanotec::PowerMode power_mode_ = Nanotec::PowerMode::OFF;
        double scale_;
        bool active_braking_ = false;
    };

}

#endif