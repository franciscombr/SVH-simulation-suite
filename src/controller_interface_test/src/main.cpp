#include "MyController_interface.h"
int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_interface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyControllerInterface ROBOT(nh);
    spinner.spin();
    
    return 0;
}
