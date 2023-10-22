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

    // Robotics Planner
    #include "robotics_planner/planner_control.h"
    #include "robotics_planner/planner_descartes.h"
    #include "robotics_planner/planner_moveitcpp.h"
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
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Planner::PlannerControl plannerControl(nh, pnh);
    // Planner::PlannerDescartes plannerDescartes(nh, pnh);

    // Trajectory::TrajectoryControl trajCtrl(nh, pnh);

    // while (ros::ok())
    // {
    //     geometry_msgs::Pose test;
    //     trajCtrl.publishMarkers(test);
    // }
    
    // PlannerMoveitCpp plannerMoveitCpp(nh);

    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}