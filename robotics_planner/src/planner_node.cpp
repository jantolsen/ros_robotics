// Planner Node 
// -------------------------------
// Description:
//      Planner Node
//
// Version:
//  0.1 - Initial Version
//        [03.01.2023]  -   Jan T. Olsen
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
    #include "robotics_planner/trajectory_ctrl.h"

// Prefix Parameter Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "planner_node");   
    
    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    // Instantiate a Joystick Control object
    Trajectory::TrajectoryControl trajCtrl(nh, pnh);
    
    // Function return
    return 0;
}