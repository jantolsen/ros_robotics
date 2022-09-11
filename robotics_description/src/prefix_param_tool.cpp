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
    #include "robotics_description/prefix_param_tool.h"

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
            ROS_ERROR("Controller Joint-Names Parameter not found!");
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
    }

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
            ROS_ERROR("Controller Joint-Limits Parameter not found!");
        }
    }

} // End Namespace