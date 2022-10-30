// Robot Joint State Publisher Node 
// -------------------------------
// Description:
//      Stream the individual motion group joint-state messages for the robot
//      Instead of using the default "joint-state-publisher" (MoveIt)
//      This node will subscribe to the robots controller-joint-states 
//      with related Robot-Prefix and publish a joint-state topic with added robot prefix. 
//      This will allow RVIZ to read the joint-states of the robots with their respective prefix name
//      This node is applicable for a multi-robot system
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
    #include <tuple>
    
    // Ros
    #include <ros/ros.h>
    #include <ros/callback_queue.h>

    // Message Types
    #include <sensor_msgs/JointState.h>
    #include <std_msgs/String.h>

    // Robotics Support
    // #include "robotics_support/prefix_param_tool.h"

// Global Variable Initialization
// -------------------------------
std::string robot_prefix;
sensor_msgs::JointState joint_state;

ros::Subscriber controller_joint_state_sub;
ros::Publisher joint_state_pub;

// int robot_count;
// std::vector<std::string> robot_prefixes;
// std::vector<std::string> robot_joint_names;
// std::vector<std::vector<std::string>> system_joint_names;

// sensor_msgs::JointState joint_state;
// std::vector<sensor_msgs::JointState> system_joint_state;
// ros::Publisher joint_state_pub;
// std::vector<ros::Publisher> joint_state_pubs;
// ros::Subscriber controller_joint_state_sub;
// std::vector<ros::Subscriber> controller_joint_state_subs;



// // Robot-Specific Subscribe and Publish Joint-States
// // -------------------------------
// // Subscribe to Controller-Joint-States and
// // publish Robot-Specific and Collective Joint-States
// void jointStateSubPub(ros::NodeHandle nh,
//                       std::string robot_prefix,
//                       std::vector<std::string> robot_joint_names)
// {
//     controller_joint_state_sub = nh.subscribe("/" + robot_prefix + "/controller_joint_states", 1000, chatterCallback);
// }

// Controller-Joint-State Callback
// -------------------------------
void jointState_Callback(const sensor_msgs::JointStatePtr& msg_)
{
    // Assign data from Controller-Joint-State Subscriber
    // to Joint-State variable
    joint_state.header = msg_->header;
    joint_state.name = msg_->name;
    joint_state.position = msg_->position;
    joint_state.velocity = msg_->velocity;
    joint_state.effort = msg_->effort;
}

// Robot Joint State Publisher Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a Anonymous ROS Node with a node name
    ros::init(argc, argv, "robot_joint_state_publisher_node");   

    // Starting the ROS Node by declaring global and private NodeHandle
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 

    // Robot Prefix 
    // -------------------------------
    // (prefix/name of the robot)
    if(!pnh.getParam("prefix", robot_prefix))
    {
        // Report to terminal
        ROS_ERROR("robot_joint_state_publisher_node: Failed to get Robot-Prefix (../robot_prefix)");

        // Function failed
        return -1;
    }

    // Subscriber
    // -------------------------------
    // Subscriber topic name
    std::string topic_name_sub = "/" + robot_prefix + "/controller_joint_states";

    // Subscribe to topic
    controller_joint_state_sub = nh.subscribe(topic_name_sub,               // Topic name
                                              1000,                         // Queue size
                                              jointState_Callback);         // Callback function

    
    // Publisher
    // -------------------------------
    // Publisher topic name
    std::string topic_name_pub = "/" + robot_prefix + "/joint_states";

    // Define publisher topic
    joint_state_pub = nh.advertise<sensor_msgs::JointState>(topic_name_pub, // Topic name
                                                            1000);          // Queue size         
    
    // ROS Loop
    // -------------------------------
    while (ros::ok())
    {
        // Publish Robot Joint-State 
        // (with Robot-prefix)
        joint_state_pub.publish(joint_state);

        // ROS Spin
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }
}