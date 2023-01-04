// Joystick Control Node 
// -------------------------------
// Description:
//      Joystick Control Node
//      for teleoperative servoing of a robot
//      Uses the connected controller and maps joystick commands
//      to related cartesian- and joint-velocities
//
// Version:
//  0.1 - Initial Version
//        [21.12.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>
    
    // Ros
    #include <ros/ros.h>

    // Robotics Joystick Control
    #include "robotics_joystick/joy_ctrl.h"

// Prefix Parameter Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "joy_ctrl_node");   
    
    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    // Instantiate a Joystick Control object
    Joystick::JoystickControl joyCtrl(nh, pnh);
    
    // Function return
    return 0;
}