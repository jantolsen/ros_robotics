// Planner Descartes 
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
#ifndef PLANNER_DESCARTES_H       
#define PLANNER_DESCARTES_H

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
    #include "robotics_planner/opw/descartes_opw_model.h"

    // Robotics Toolbox
    #include "robotics_toolbox/toolbox.h"

    // Robotics Planner
    #include "robotics_planner/planner_tool.h"


// Namespace: Planner
// -------------------------------
namespace Planner
{
    // Planner Descartes Class
    // -------------------------------
    class PlannerDescartes
    {
        // Public Class members
        // -------------------------------
        // Accessible for everyone
        public:
            // Class constructor
            // -------------------------------
            PlannerDescartes(
                ros::NodeHandle& nh,
                ros::NodeHandle& pnh);

            // Class destructor
            // -------------------------------
            ~PlannerDescartes();


        // Protected Class members
        // -------------------------------
        // Accessible within the class which defines them, 
        // and classes which inherits from the parent class
        protected:

            // Member Variables
            // -------------------------------
            ros::NodeHandle nh_;        // ROS Nodehandle
            ros::NodeHandle pnh_;       // ROS Private Nodehandle
            Planner::Config config_;    // General configuration for descartes control

            // ROS Publisher(s)
            // -------------------------------
            ros::Publisher pub_marker_trajectory_;  // Publisher for Trajectory visualization markers (RVIZ)

            // ROS Action Client(s)
            // -----------------------
            std::shared_ptr<Planner::ExecuteTrajectoryActionClient> ptr_exec_trajectory_ac_; // Sends a robot trajectory to move-it for execution

            // Descartes Constructs
            // -----------------------
            descartes_core::RobotModelPtr robot_model_;
            

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
            std::vector<descartes_core::TrajectoryPtPtr> computeTrajectory();


            // Plan Descartes Trajectory
            // -------------------------------
            /** \brief Plan given Joint Trajectory
            */
            std::vector<descartes_core::TrajectoryPtPtr> planTrajectory(
                const std::vector<descartes_core::TrajectoryPtPtr>& trajectory);


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
            const double ACTIONSERVER_TIMEOUT = 1.0;    // Timeout parameter for Action-Server [sec]
            const int QUEUE_LENGTH = 1;                 // Maximum number of outgoing messages to be queued for delivery
            const std::string CLASS_PREFIX = "PlannerDescartes::";   // Class message-prefix for terminal output


            // Load Kinematics Parameters
            // -------------------------------
            /** \brief Load Kinematics Parameters
            * Based on the robot-type, find the corresponding opw kinematics parameters
            * using predefined Kinematics-Parameters function
            * \param robot_type Robot-Type [std::string]
            * \param kinematic_parameters Robot OPW Kinematics Parameters [opw_kinematics::Parameters<double>]
            * \return Function result: Successful/Unsuccessful (true/false)
            */
            bool loadKinematicParameters(
                const std::string robot_type,
                opw_kinematics::Parameters<double>& kinematic_parameters)
            {
                // ABB IRB6660
                if(robot_type == "irb6660")
                {
                    // Assign Kinematic-Parameters    
                    kinematic_parameters = opw_kinematics::makeABB_IRB6660<double>();

                    // Function return
                    return true;
                }
                
                // Yaskawa GP400
                if(robot_type == "gp400")
                {
                    // Assign Kinematic-Parameters    
                    kinematic_parameters = opw_kinematics::makeDefaultRobot<double>();

                    // Function return
                    return true;
                }
                
                // No listed Robot-Type was found
                return false;
            }
            
    }; // End Class: Planner Descartes
} // End Namespace: Planner
#endif // PLANNER_DESCARTES_H 