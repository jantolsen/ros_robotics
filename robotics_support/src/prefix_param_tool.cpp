// Toolbox Prefix Parameter 
// -------------------------------
// Description:
//      Toolbox for Prefix Paramters
//      Functions for loading and remapping robot specific parameters to correct for robot prefix
//      Applicaple for when having multiple robots in the same scene.
//
// Version:
//  0.1 - Initial Version
//        [10.09.2022]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Main Header-File
    #include "robotics_support/prefix_param_tool.h"

// Namespace: Prefix Param Toolbox
// -------------------------------
namespace PrefixParamTool
{

    // Prefix Joint-Names
    // -------------------------------
    void prefixJointNames(ros::NodeHandle nh,
                          std::string robot_prefix)
    {   
        // Defining local variables
        std::string joint_name;
        std::vector<std::string> joint_names;
        std::vector<std::string> prefix_joint_names;

        // Check parameter server for existing controller joint-names
        if(nh.getParam("controller_joint_names", joint_names))
        {
            // Parameter Server's Joint-Names are defined as a list

            // Iterate over each joint-names
            for (std::size_t i = 0; i < joint_names.size(); i ++)
            {
                // Joint Name
                joint_name = robot_prefix + "_" + joint_names[i];

                // Assign robot-prefix to each joint-name
                prefix_joint_names.push_back(joint_name); 
            }
        }

        // Parameter not found on parameter server
        else
        {
            // Report to terminal
            ROS_ERROR("prefixJointNames: Controller Joint-Names Parameter not found!");
        }

        // Check for existing controller joint-names on global parameter server
        if(nh.getParam("/controller_joint_names", joint_names))
        {
            // Iterate over each joint-name
            for (std::size_t i = 0; i < prefix_joint_names.size(); i ++)
            {
                // Append newly assign joint-names to existing list
                joint_names.push_back(prefix_joint_names[i]); 
            }
            
            // Create new joint-names on global parameter server
            nh.setParam("/controller_joint_names", joint_names);
        }
        
        // No existing joint-names on global parameter server
        else
        {
            // Create new joint-names on global parameter server
            nh.setParam("/controller_joint_names", prefix_joint_names);
        }

        // Create new robot specific joint-names on global parameter server
        nh.setParam("/" + robot_prefix + "/controller_joint_names", prefix_joint_names);

        // Delete private joint-names parameter on the anonymous nodehandle
        nh.deleteParam("controller_joint_names");

    } // End-Function: Prefix Joint-Names

    
    // Prefix Joint-Limits
    // -------------------------------
    void prefixJointLimits(ros::NodeHandle nh,
                           std::string robot_prefix)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue joint_limits;
        bool has_velocity_limits;
        double max_velocity;
        bool has_acceleration_limits;
        double max_acceleration;

        // Check parameter server for existing controller joint-names
        if(nh.getParam("joint_limits", joint_limits))
        {
            // Parameter Server's Joint-Limits are defined as a XmlRpc-type
            // (size = 5, starting at 0-index)

            // Iterate over each joint of Joint-Limits
            for (std::size_t i = 1; i < (joint_limits.size() + 1); i ++)
            {
                // Get Parameters
                // -------------------------------
                    // Has Velocity Limit
                    nh.getParam("joint_limits/joint_" + std::to_string(i) + 
                                "/has_velocity_limits", has_velocity_limits);

                    // Max Velocity
                    nh.getParam("joint_limits/joint_" + std::to_string(i) + 
                                "/max_velocity", max_velocity);

                    // Has Acceleration Limit
                    nh.getParam("joint_limits/joint_" + std::to_string(i) + 
                                "/has_acceleration_limits", has_acceleration_limits);

                    // Max Velocity
                    nh.getParam("joint_limits/joint_" + std::to_string(i) + 
                                "/max_acceleration", max_acceleration);

                // Create new Parameters with robot prefix
                // -------------------------------
                    // Has Velocity Limit
                    nh.setParam("/robot_description_planning/joint_limits/" + 
                                robot_prefix + "_" + "joint_" + std::to_string(i) + 
                                "/has_velocity_limits", has_velocity_limits);

                    // Max Velocity
                    nh.setParam("/robot_description_planning/joint_limits/" + 
                                robot_prefix + "_" +  "joint_" + std::to_string(i) + 
                                "/max_velocity", max_velocity);

                    // Has Acceleration Limit
                    nh.setParam("/robot_description_planning/joint_limits/" + 
                                robot_prefix + "_" +  "joint_" + std::to_string(i) + 
                                "/has_acceleration_limits", has_acceleration_limits);

                    // Max Velocity
                    nh.setParam("/robot_description_planning/joint_limits/" + 
                                robot_prefix + "_" + "joint_" + std::to_string(i) + 
                                "/max_acceleration", max_acceleration);
            }

            // Delete private joint-limits parameter on the anonymous nodehandle
            nh.deleteParam("joint_limits");
        }

        // Parameter not found on parameter server
        else
        {
            // Report to terminal
            ROS_ERROR("prefixJointLimits: Controller Joint-Limits Parameter not found!");
        }

    } // End-Function: Prefix Joint-Limits


    // Prefix Controller-List
    // -------------------------------
    void prefixControllerList(ros::NodeHandle nh,
                              std::string robot_prefix)
    {
        
        // Defining local variables 
        XmlRpc::XmlRpcValue controller_list;
        XmlRpc::XmlRpcValue robot_controller;
        XmlRpc::XmlRpcValue joint_names;

        // Check parameter server for existing robot specific controller-joint-names parameter
        if(!nh.getParam("/" + robot_prefix + "/controller_joint_names", joint_names))
        {
            ROS_ERROR("prefixControllerList: Failed to get Controller-Joint-Names for Robot (%s)", robot_prefix.c_str());
        }

        // Define robot specific Robot-Controller parameters
        robot_controller["name"] = robot_prefix;
        robot_controller["action_ns"] = "joint_trajectory_action";
        robot_controller["type"] = "FollowJointTrajectory";
        robot_controller["joints"] = joint_names;

        // Check for existing controller-list on global parameter server
        if(nh.getParam("/move_group/controller_list", controller_list))
        {
            // Set robot index equal to the number of independent robots found within the controller-list parameter
            int robot_index = controller_list.size();

            // Append the new robot-controller parameters to the existing controller-list parameter
            controller_list[robot_index] = robot_controller;

            // Create new joint-names on global parameter server
            nh.setParam("/move_group/controller_list", controller_list);
        }
        
        // No existing controller-list on global parameter server
        else
        {
            // Append the new robot-controller parameters to the controller-list parameter
            controller_list[0] = robot_controller;

            // Create new controller-list parameters on global parameter server
            nh.setParam("/move_group/controller_list", controller_list);
        }
        
    } // End-Function: Prefix Controller-List


    // Prefix Topic-List
    // -------------------------------
    void prefixTopicList(ros::NodeHandle nh,
                         std::string robot_prefix)
    {
        
        // Defining local variables 
        XmlRpc::XmlRpcValue topic_list;
        XmlRpc::XmlRpcValue robot_topic;
        XmlRpc::XmlRpcValue joint_names;

        // Check parameter server for existing robot specific controller-joint-names parameter
        if(!nh.getParam("/" + robot_prefix + "/controller_joint_names", joint_names))
        {
            ROS_ERROR("prefixTopicList: Failed to get Controller-Joint-Names for Robot (%s)", robot_prefix.c_str());
        }

        // Define robot specific Robot-Controller parameters
        robot_topic["name"] = robot_prefix + "controller";
        robot_topic["ns"] = robot_prefix;
        robot_topic["group"] = 0;
        robot_topic["joints"] = joint_names;

        // Append the new robot-controller parameters to the controller-list parameter
        topic_list[0] = robot_topic;

        // Create new topic-list parameters on global parameter server
        nh.setParam("/" + robot_prefix + "/topic_list", topic_list);

    } // End-Function: Prefix Topic-List


    // Prefix Kinematics Parameters
    // -------------------------------
    void prefixKinematicsParam(ros::NodeHandle nh,
                               std::string robot_prefix)
    {
        // Defining local variables 
        XmlRpc::XmlRpcValue kinematics_param;
        
        // Check parameter server for existing kinematics parameters
        if(nh.getParam("default_manipulator", kinematics_param))
        {
            // Create new kinematics parameters on global parameter server
            nh.setParam("/robot_description_kinematics/" + robot_prefix + "_manipulator", kinematics_param);

            // Delete private kinematics parameter on the anonymous nodehandle
            nh.deleteParam("default_manipulator");
        }

        // Parameter not found on parameter server
        else
        {
            // Report to terminal
            ROS_ERROR("prefixKinematicsParam: Kinematics Parameters not found!");
        }

    } // End-Function: Prefix Kinematics Parameter

} // End Namespace