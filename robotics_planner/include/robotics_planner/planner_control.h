// Planner Control 
// -------------------------------
// Description:
//      
//      
//
// Version:
//  0.1 - Initial Version
//        [19.01.2023]  -   Jan T. Olsen
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
#ifndef PLANNER_CONTROL_H       
#define PLANNER_CONTROL_H

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
    #include <robotics_planner/opw/moveit_opw_kinematics_plugin.h>
    #include <robotics_planner/opw/opw_parameters_examples.h>

    // Descartes
    #include <descartes_planner/dense_planner.h>
    #include <descartes_planner/sparse_planner.h>
    #include <descartes_trajectory/axial_symmetric_pt.h>
    #include <descartes_trajectory/cart_trajectory_pt.h>
    #include <descartes_utilities/ros_conversions.h>

    #include <descartes_moveit/moveit_state_adapter.h>
    #include <descartes_moveit/ikfast_moveit_state_adapter.h>

    #include "robotics_planner/opw/descartes_opw_model.h"

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
            std::vector<std::string> joint_names;   // Robot joint names
        };

    // Type definitions
    // -------------------------------
        typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ExecuteTrajectoryActionClient;

    // Planner Control Class
    // -------------------------------
    class PlannerControl
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Class constructor
            // -------------------------------
            PlannerControl(
                ros::NodeHandle& nh,
                ros::NodeHandle& pnh);

            // Class destructor
            // -------------------------------
            ~PlannerControl();

        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Member Variables
            // -------------------------------
            ros::NodeHandle nh_;                            // ROS Nodehandle
            ros::NodeHandle pnh_;                           // ROS Private Nodehandle
            Config config_;                                 // General configuration data for planner control
            std::string class_prefix_ = "PlannerControl:";  // Class message-prefix for terminal output

            // ROS Publisher(s)
            // -------------------------------
            ros::Publisher pub_marker_trajectory_;          // Publisher for Trajectory visualization markers (RVIZ)

            // ROS Action Client(s)
            // -----------------------
            std::shared_ptr<ExecuteTrajectoryActionClient> ptr_exec_trajectory_ac_; // Sends a robot trajectory to move-it for execution

            // Descartes Constructs
            // -----------------------
            descartes_core::RobotModelPtr ptr_robot_model_;
            
            // Class initialiation
            // -------------------------------
            /** \brief Initialization of PlannerControl class.
            * Called from class constructor
            */
            void init();

            // Load Parameters
            // -------------------------------
            /** \brief Load Parameters from parameter server
            * Called from init-function
            */
            void loadParameters();

            // Initialize Publisher(s)
            // -------------------------------
            /** \brief Collective function for initialization of Publisher(s)
            * Called from init-function
            */
            void initPublishers();

            // Initialize Action-Client(s)
            // -------------------------------
            /** \brief Collective function for initialization of Action-Client(s)
            * Called from init-function
            */
            void initActionClients();

            // Create and Initialize Robot-Model
            // -------------------------------
            /** \brief Collective function for the creation and initialization of Robot-Model
            * Called from init-function
            */
            void initRobotModel();

            // Compute Descartes Trajectory
            // -------------------------------
            /** \brief Compute a Descartes Trajectory
            */
            void computeDescartesTrajectory();

            // Execute Trajectory
            // -------------------------------
            /** \brief Execute given Joint Trajectory
            */
            void executeTrajectory(
                const std::vector<descartes_core::TrajectoryPtPtr>& trajectory);

        // Private Class members
        // -------------------------------
        // Accessible only for the class which defines them
        private:

            // Constants
            // -------------------------------
            const double ACTIONSERVER_TIMEOUT = 5.0;    // Timeout parameter for Action-Server [sec]
            const int QUEUE_LENGTH = 1;                 // Maximum number of outgoing messages to be queued for delivery
    };

} // End Namespace: Planner Toolbox
#endif // PLANNER_CONTROL_H 