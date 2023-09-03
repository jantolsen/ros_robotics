// Planner MoveIt-Cpp 
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
#ifndef PLANNER_MOVEITCPP_H       
#define PLANNER_MOVEITCPP_H

// Include Header-files:
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>
    #include <map>

    // Ros
    #include <ros/ros.h>

    // MoveItCpp
    #include <moveit/moveit_cpp/moveit_cpp.h>
    #include <moveit/moveit_cpp/planning_component.h>
    #include <moveit/robot_state/conversions.h>
    #include <moveit_visual_tools/moveit_visual_tools.h>

    // Messages
    #include <geometry_msgs/PointStamped.h>

// Planner Moveit-Cpp Class
// -------------------------------
class PlannerMoveitCpp
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:
        // Class constructor
        // -------------------------------
        PlannerMoveitCpp(
            ros::NodeHandle& nh);

        // Class destructor
        // -------------------------------
        ~PlannerMoveitCpp();

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected: 
        // Member Variables
        // -------------------------------
        ros::NodeHandle nh;        // ROS Nodehandle

        // Class execution
        // -------------------------------
        /** \brief Initialization and execution of class.
        * Called from class constructor
        */
        void exec();

    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Constants
        // -------------------------------
        const std::string CLASS_PREFIX = "PlannerMoveitCpp::";   // Class message-prefix for terminal output
        const std::string PLANNING_GROUP = "robot_manipulator";


}; // End Class: Planner Descartes
#endif // PLANNER_MOVEITCPP_H 