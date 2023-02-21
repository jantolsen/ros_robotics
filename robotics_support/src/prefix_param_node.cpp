// Prefix Parameter Node 
// -------------------------------
// Description:
//      Loads and remaps robot specific parameters to correct for robot prefix.
//      Applicaple for when having multiple robots in the same scene.
//      The node finds the robot's respective parameter yaml-file(s),
//      adds the prefix name, and loads them as parameters to the parameter server
//
// Version:
//  0.1 - Initial Version
//        [10.09.2022]  -   Jan T. Olsen
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

    // Robotics Support
    #include "robotics_support/prefix_param_tool.h"

// Prefix Parameter Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
        // Initialize a Anonymous ROS Node with a node name
        ros::init(argc, argv, "prefix_parameter_node", ros::init_options::AnonymousName);   
        
        // Starting the ROS Node by declaring private NodeHandle
        ros::NodeHandle nh("~"); 

        // Defining local members
        std::string robot_prefix;
        std::string robot_type;

        // Get Robot-Prefix parameter from Parameter Server
        nh.getParam("robot_prefix", robot_prefix);

        // Get Robot-Type parameter from Parameter Server
        nh.getParam("robot_type", robot_type);

    // Prefix Robot Joint-Names Parameters
    // -------------------------------
    // Get Joint-Name parameter (loaded together with node)
    // and assigned them according to robot prefix
        PrefixParamTool::prefixJointNames(nh, robot_prefix);

    // Prefix Robot Joint-Limits Parameters
    // -------------------------------
    // Get Joint-Limits parameter (loaded together with node)
    // and assigned them according to robot prefix
        PrefixParamTool::prefixJointLimits(nh, robot_prefix);

    // Prefix Robot Controller-List Parameters
    // -------------------------------
    // Create Controller-List parameters
    // and assigned them according to robot prefix
        PrefixParamTool::prefixControllerList(nh, robot_prefix);

    // Prefix Robot Topic-List Parameters
    // -------------------------------
    // Create Topic-List parameters
    // and assigned them according to robot prefix
        PrefixParamTool::prefixTopicList(nh, robot_prefix, robot_type);

    // Prefix Robot Kinematic Parameters
    // -------------------------------
    // Get Kinematic parameter (loaded together with node)
    // and assigned them according to robot prefix
        PrefixParamTool::prefixKinematicsParam(nh, robot_prefix);

    // Prefix OMPL-Planning-Parameters
    // -------------------------------
    // Remap robot's OMPL-Planning parameters
    // (loaded together with planning-pipeline), 
    // and assign them according to robot prefix
        PrefixParamTool::prefixOMPLParam(nh, robot_prefix);

    // Delete private robot-prefix parameter on the anonymous nodehandle
    nh.deleteParam("robot_prefix");
}