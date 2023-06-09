// Planner Toolbox 
// -------------------------------
// Description:
//      Toolbox for Planner
//      Contains several helper and utility functions
//
// Version:
//  0.1 - Initial Version
//        [05.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef PLANNER_TOOL_H       
#define PLANNER_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>
    #include <actionlib/client/simple_action_client.h>

    // MoveIt
    #include <moveit_msgs/ExecuteTrajectoryAction.h>

    // Robotics Toolbox
    #include <robotics_toolbox/toolbox.h>

// Namespace: Planner
// -------------------------------
namespace Planner
{
    // Constants
    // -------------------------------
        const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
        const std::string EXECUTE_TRAJECTORY_ACTION = "execute_trajectory";
        const std::string ROBOT_PREFIX = "robot";
        const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualization_trajectory";

    // Structs
    // -------------------------------
        // Config    
        struct Config
        {
            std::string robot_prefix;               // Robot prefix name
            std::string group_name;                 // Robot manipulation group container 
            std::string global_frame;               // World-link frame
            std::string robot_frame;                // Robot base-link frame
            std::string tool_frame;                 // Robot tool-link frame
            std::string robot_type;                 // Robot type 
            std::vector<std::string> joint_names;   // Robot joint names
        };

    // Enums
    // -------------------------------
        

    // Type definitions
    // -------------------------------
        typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ExecuteTrajectoryActionClient;

    // Functions
    // -------------------------------
        
        


} // End Namespace: Planner
#endif // PLANNER_TOOL_H 