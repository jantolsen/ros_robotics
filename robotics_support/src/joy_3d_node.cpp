// 3D-Mouse Publisher Node 
// -------------------------------
// Description:
//      Based on "spacenav_to_twist" 
//      http://wiki.ros.org/spacenav_node
//
// Version:
//  0.1 - Initial Version
//        [18.12.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    
    // Ros
    #include <ros/ros.h>

    // Message Types
    #include "geometry_msgs/TwistStamped.h"
    #include "control_msgs/JointJog.h"
    #include "sensor_msgs/Joy.h"

// Namespace: Joy 3D-Mouse
// -------------------------------
namespace joy_teleop
{
    static const int NUM_SPINNERS = 1;
    static const int QUEUE_LENGTH = 1;


    // Class: SpacePilot 3D-Mouse
    // -------------------------------
    // Initializes Subscriber and Publisher for
    // SpacePilot 3D-Mouse
    class SpacePilot
    {   
        // Public Class members
        // (Accessible for everyone)
        public:

            // Class Constructor
            // -------------------------------
            SpacePilot(ros::NodeHandle nh,
                       ros::NodeHandle pnh)
                : spinner_(NUM_SPINNERS),
                  nh_(nh),
                  pnh_(pnh)
            {
                // Define Joystick (input) Subscriber
                joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",                       // Topic name
                                                           QUEUE_LENGTH,                // Queue size
                                                           &SpacePilot::joyCallback,    // Callback function
                                                           this);                       // Class Object
                
                // Define Cartesian Publisher
                cartestian_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("servo_server/delta_twist_cmds",   // Topic name
                                                                             QUEUE_LENGTH);                     // Queue size  

                // Define Cartesian Publisher
                joint_pub_ = nh_.advertise<control_msgs::JointJog>("servo_server/delta_joint_cmds",             // Topic name
                                                                    QUEUE_LENGTH);                              // Queue size  
                                                                             
                // Get Robot Joint-Names
                getRobotJointNames();

                // Start Async-Spinner
                spinner_.start();
                ros::waitForShutdown();
            }

            // Class Destructor
            // -------------------------------
            ~SpacePilot()
            {

            }


        // Private Class members
        // (Accessible only for the class which defines them)
        private:

            // Class Members
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_; 
            ros::Subscriber joy_sub_;
            ros::Publisher cartestian_pub_;
            ros::Publisher joint_pub_;
            ros::AsyncSpinner spinner_;
            std::vector<std::string> joint_names_;

            // Joystick Callback
            // -------------------------------
            // Convert incoming Joy-commands to TwistedStamped-commands
            void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
            {
                // Cartesian servoing with the axes
                // -------------------------------
                    // Define temporary variable holder
                    geometry_msgs::TwistStamped twist;
                    
                    // Map data to message-type
                    twist.header.stamp = ros::Time::now();
                    twist.twist.linear.x = msg->axes[0];
                    twist.twist.linear.y = msg->axes[1];
                    twist.twist.linear.z = msg->axes[2];

                    twist.twist.angular.x = msg->axes[3];
                    twist.twist.angular.y = msg->axes[4];
                    twist.twist.angular.z = msg->axes[5];

                    // Publish Cartesian servoing commands
                    cartestian_pub_.publish(twist);

                // Joint servoing with the buttons
                // -------------------------------
                    // Define temporary variable holder
                    control_msgs::JointJog joint_deltas;
                    
                    // Map data to message-type
                    joint_deltas.header.stamp = ros::Time::now();

                    // Joint-Names
                    for (int i = 0; i < joint_names_.size(); i++)
                    {
                        joint_deltas.joint_names.push_back(joint_names_[i]);
                    }

                    // Joint-Velocity commands
                    joint_deltas.velocities.push_back(msg->buttons[0] - msg->buttons[1]);       // Joint 1
                    joint_deltas.velocities.push_back(msg->buttons[22] - msg->buttons[28]);     // Joint 2
                    joint_deltas.velocities.push_back(msg->buttons[25] - msg->buttons[27]);     // Joint 3
                    joint_deltas.velocities.push_back(msg->buttons[23] - msg->buttons[26]);     // Joint 4
                    joint_deltas.velocities.push_back(msg->buttons[24] - msg->buttons[29] - msg->buttons[30]);     // Joint 5
                    joint_deltas.velocities.push_back(msg->buttons[2] - msg->buttons[3]);       // Joint 6

                    // Publish Joint servoing commands
                    joint_pub_.publish(joint_deltas);
            }
    
        // Get Robot Joint-Names
        // -------------------------------
        void getRobotJointNames()
        {
            // Defining local members
            std::string robot_prefix;

            // Get Robot-Prefix parameter from Parameter Server
            if(!pnh_.getParam("robot_prefix", robot_prefix))
            {
                // Report to terminal
                ROS_ERROR("joy_3d_node: Failed to get Robot-Prefix");

                // Function failed
                return;
            }

            // Get Robot Joint-Names from Parameter Server
            if(!pnh_.getParam("/" + robot_prefix + "/controller_joint_names", joint_names_))
            {
                // Report to terminal
                ROS_ERROR("joy_3d_node: Failed to get Robot-Joint-Names for Robot-Prefix (%s)", robot_prefix.c_str());

                // Function failed
                return;
            }
        }
    };
} // Namespace: Joy 3D-Mouse
// -------------------------------

// Joystick 3D-Mouse Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a Anonymous ROS Node with a node name
    ros::init(argc, argv, "joy_3d_node");

    // Starting the ROS Node by declaring global and private NodeHandle
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 

    // Define Joy 3D-Mouse object
    joy_teleop::SpacePilot joy(nh, pnh);

    // Function return
    return 0;
}

    