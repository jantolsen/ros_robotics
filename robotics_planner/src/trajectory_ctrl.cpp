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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_planner/trajectory_ctrl.h"

// Namespace: Trajectory
// -------------------------------
namespace Trajectory
{

    // Class constructor
    // -------------------------------
    TrajectoryControl::TrajectoryControl(
        ros::NodeHandle& nh,
        ros::NodeHandle& pnh)
    :
        nh_(nh),
        pnh_(pnh),
        asyncspinner_(NUM_SPINNERS)
    {
        // Initialize
        init();
        
        // Start Async-Spinner
        asyncspinner_.start();
        ros::waitForShutdown();
    }

    // Class destructor
    // -------------------------------
    TrajectoryControl::~TrajectoryControl()
    {
        // Report to terminal
        ROS_INFO_STREAM(msg_prefix_ + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }

    // Class initialiation
    // -------------------------------
    void TrajectoryControl::init()
    {
        
    }
} // Namespace: Trajectory