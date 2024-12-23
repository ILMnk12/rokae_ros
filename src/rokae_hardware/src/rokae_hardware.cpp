
#include <vector>
#include <string>
#include <thread>

#include <sensor_msgs/JointState.h>

#include "rokae_hardware/rokae_hardware.h"

#include <controller_manager/controller_manager.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "rokae_hardware_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();    

    ros::NodeHandle nh;
    std::error_code ec;

    ros::Publisher record = nh.advertise<sensor_msgs::JointState>("/record",1000);

    // instance for 7-axis robot only.
    // if 6-axis robot is connected, try the command underline.
    // rokae_hardware::RokaeHardwareInterface<6> rokae_hardware_interface;   
    
    rokae_hardware::RokaeHardwareInterface<6> rokae_hardware_interface;

    ros::Time timestamp;
    ros::Duration period;


    ros::Duration period_(0.001);


    if (!rokae_hardware_interface.init(nh)) {
        ROS_ERROR("Failed to initialize rokae Hardware Interface.");
    }

    controller_manager::ControllerManager cm(&rokae_hardware_interface, nh);

    ros::Rate rate(1000);

    ros::TimerEvent e; 

    while(ros::ok())
    {
        // Read the current state from hardware
        rokae_hardware_interface.read();
        timestamp = ros::Time::now();
        // Update the controller manager
        cm.update(timestamp, period_);

        // Write the hardware
        rokae_hardware_interface.write(period_);
  
        rate.sleep();
    }

    spinner.stop();

    ROS_INFO_STREAM_NAMED("rokae_hardware_interface", "Shutting down.");

    return 0;


}
