#include <aleph2_hardware_interface/aleph2_hw.h>
#include <controller_manager/controller_manager.h>

#include "std_msgs/Float32.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_manager");

    ros::NodeHandle nh("~");

    std::string hw_namespace;
    int spinner_threads, loop_rate;

    nh.param("hw_namespace", hw_namespace, hw_namespace);
    nh.param("spinner_threads", spinner_threads, 4);
    nh.param("loop_rate", loop_rate, 20);

    std_msgs::Float32 usage_msg;
    ros::Publisher usage_pub = nh.advertise<std_msgs::Float32>("usage", 5);

    aleph2_hardware_interface::Aleph2HW aleph2_hw;
    ros::NodeHandle hw_nh(hw_namespace);
    aleph2_hw.init(hw_nh);

    controller_manager::ControllerManager cm(&aleph2_hw, hw_nh);

    ros::AsyncSpinner spinner(spinner_threads);
    ros::Rate rate(loop_rate);

    ros::Time last_update = ros::Time::now();
    spinner.start();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        aleph2_hw.read();
        cm.update(current_time, current_time - last_update);
        aleph2_hw.write(current_time - last_update);

        last_update = current_time;
        rate.sleep();

        usage_msg.data = (rate.cycleTime().toSec() / rate.expectedCycleTime().toSec()) * 100;
        usage_pub.publish(usage_msg);
    }
}