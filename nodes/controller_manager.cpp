#include <aleph2_hardware_interface/aleph2_hardware_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_manager");

    aleph2_hardware_interface::Aleph2HardwareInterface aleph2;
    ros::NodeHandle robot_hw_nh("aleph2");
    aleph2.init(robot_hw_nh);

    controller_manager::ControllerManager cm(&aleph2);

    ros::AsyncSpinner spinner(1);
    ros::Rate rate(20);

    ros::Time last_update = ros::Time::now();
    spinner.start();
    
    while (ros::ok())
    {
        //ROS_INFO("DUPA");
        ros::Time current_time = ros::Time::now();

        aleph2.read();
        cm.update(current_time, current_time - last_update);
        aleph2.write();

        last_update = current_time;
        rate.sleep();
    }
}