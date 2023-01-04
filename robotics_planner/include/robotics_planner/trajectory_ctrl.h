// Trajectory Controll
// ----------------------------------------------
// Description:
//      Trajectory Control and Planner
//
// Version:
//  0.1 - Initial Version
//        [04.01.2023]  -   Jan T. Olsen
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
#ifndef TRAJECTORY_CONTROL_H       
#define TRAJECTORY_CONTROL_H   

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <string>
    #include <map>
    
    // ROS
    #include <ros/ros.h>

    // Messages
    #include "geometry_msgs/TwistStamped.h"

// Namespace: Trajectory
// -------------------------------
namespace Trajectory
{
// Trajectory Control Class
// -------------------------------
class TrajectoryControl
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Class constructor
        // -------------------------------
        TrajectoryControl(
            ros::NodeHandle& nh,
            ros::NodeHandle& pnh);

        // Class destructor
        // -------------------------------
        ~TrajectoryControl();

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:
        // Member Variables
        // -------------------------------
        ros::NodeHandle nh_;                // ROS Nodehandle
        ros::NodeHandle pnh_;               // ROS Private Nodehandle
        ros::AsyncSpinner asyncspinner_;    // ROS Asynchronous Spinner

        std::string msg_prefix_ = "TrajectoryCtrl: ";   // Class message-prefix for termnial output

        // Class initialiation
        // -------------------------------
        /** \brief Initialization of Joy-Ctrl class.
        * Called from class constructor
        */
        void init();

    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Constants
        static const int NUM_SPINNERS = 1;
        static const int QUEUE_LENGTH = 1;
};

}   // Namespace: Trajectory
#endif //TRAJECTORY_CONTROL_H