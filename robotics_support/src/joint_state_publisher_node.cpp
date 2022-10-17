// Joint State Publisher Node 
// -------------------------------
// Description:
//      Stream the individual motion group joint-state messages for the robot(s)
//      This node is applicable for a multi-robot system
//      This will allow RVIZ to read the joint-states of the robots with their
//      respective prefix name
//
// Version:
//  0.1 - Initial Version
//        [16.10.2022]  -   Jan T. Olsen
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
    #include <sensor_msgs/JointState.h>

    // Robotics Support
    // #include "robotics_support/prefix_param_tool.h"

// Global Variable Initialization
// -------------------------------
int robot_count;
std::vector<std::string> robot_prefixes;
std::vector<std::string> joint_names;
std::vector<std::vector<std::string>> system_joint_names;
sensor_msgs::JointState joint_state;
std::vector<sensor_msgs::JointState> joint_states;
ros::Publisher joint_state_pub;
std::vector<ros::Publisher> joint_state_pubs;
std::vector<ros::Subscriber> controller_joint_states_subs;

// Get System Parameters
// -------------------------------
// (search on parameter server for system parameters)
void getSystemParam(ros::NodeHandle pnh)
{
    // Robot Count 
    // -------------------------------
    // (number of robots in the system)
    if(!pnh.getParam("/general/robot_count", robot_count))
    {
        // Report to terminal
        ROS_ERROR("joint_state_publisher_node: Failed to get Robots-Count (/general/robot_count)");

        // Function failed
        return;
    }

    // Robot Prefix 
    // -------------------------------
    // (prefix/name of the robots)
    if(!pnh.getParam("/general/robot_prefixes", robot_prefixes))
    {
        // Report to terminal
        ROS_ERROR("joint_state_publisher_node: Failed to get Robots-Prefixes (/general/robot_prefixes)");

        // Function failed
        return;
    }
    
    // Validation check 
    // -------------------------------
    // (robot-count and number of robot-prefixes)
    if(robot_count != robot_prefixes.size())
    {
        // Report to terminal
        ROS_ERROR("joint_state_publisher_node: Robot-Count (%i) differs from number of Robot-Prefixes (%zd)", robot_count, robot_prefixes.size());

        // Function failed
        return;
    }

    // Robot Joint-Names
    // -------------------------------
    // (iterate through number of robots and get joint-names) 
    for(int i = 0; i < robot_count; i ++)
    {
        // Get Joint-Names for Robot [i]
        if(!pnh.getParam("/" + robot_prefixes[i] + "/controller_joint_names", joint_names))
        {
            // Report to terminal
            ROS_ERROR("joint_state_publisher_node: Failed to get Robots-Controller-Joint-Names (/%s/controller_joint_names)", robot_prefixes[i].c_str());

            // Function failed
            return;
        }

        // Push acquired Joint-Names to collective system joint-names
        system_joint_names.push_back(joint_names);

        // Clear Joint-Name variable holder for next iteration
        joint_names.clear();
    }

    // // Debug
    // // -------------------------------
    // for(int i = 0; i < robot_count; i ++)
    // {
    //     // Robot-Prefix[i]
    //     ROS_ERROR("ROBOT PREFIX NO (%i): %s", i, robot_prefixes[i].c_str());

    //     for(int k = 0; k < system_joint_names[i].size(); k ++)
    //     {
    //         // Robot-Prefix Joint-Name[k]
    //         ROS_ERROR(" JOINT-NAME (%s)", system_joint_names[i][k].c_str());
    //     }

    //     ROS_ERROR("----------------");
    // }
}

// Joint State Publisher Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
        // Initialize a Anonymous ROS Node with a node name
        ros::init(argc, argv, "joint_state_publisher_node");   

        // Starting the ROS Node by declaring global and private NodeHandle
        ros::NodeHandle nh; 
        ros::NodeHandle pnh("~"); 

        // Get system parameters from parameter-server
        getSystemParam(pnh);
}