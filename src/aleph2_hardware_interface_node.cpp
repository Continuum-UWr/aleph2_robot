#include <aleph2_hardware_interface/aleph2_hardware_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ROBOT_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    aleph2_hardware_interface::Aleph2HardwareInterface aleph2(nh);
    ros::spin();
    return 0;
}