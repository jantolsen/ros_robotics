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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_planner/planner_control.h"

// Namespace: Planner
// -------------------------------
namespace Planner
{

    // Class constructor
    // -------------------------------
    PlannerControl::PlannerControl(
        ros::NodeHandle& nh,
        ros::NodeHandle& pnh)
    :
        nh_(nh),
        pnh_(pnh)
    {
        // Initialize
        init();
    }

    // Class destructor
    // -------------------------------
    PlannerControl::~PlannerControl()
    {
        // Report to terminal
        ROS_INFO_STREAM(class_prefix_ + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }    

    // Class initialization
    // -------------------------------
    void PlannerControl::init()
    {
        // Load parameters
        loadParameters();

        // Execute-Trajectory Action-Client
        // -------------------------------
        // Define the action-client as a shared pointer
        ptr_exec_trajectory_ac_ = std::make_shared<ExecuteTrajectoryActionClient>(EXECUTE_TRAJECTORY_ACTION, true);

        // Establish connection to server
        if(!ptr_exec_trajectory_ac_->waitForServer(ros::Duration(ACTIONSERVER_TIMEOUT)))
        {
            // Report timeout and a failed connection
            ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                            ": Timeout! Failed to establish connection with Action-Client: " << EXECUTE_TRAJECTORY_ACTION ); 

            // Exit
            exit(-1);
        }

        // Successful established connection
        ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Sucessfully established connection with Action-Client: " << EXECUTE_TRAJECTORY_ACTION); 

    }

    // Load Parameters
    // -------------------------------
    void PlannerControl::loadParameters()
    {
        // Read robot-prefix
        if(!pnh_.getParam("/general/robot_prefix/robot_1", config_.robot_prefix))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(class_prefix_ + "Failed to get parameter (/robot_prefix) from server");

            // Exit
            exit(-1);
        }

        // Read global frame data
        if(!pnh_.getParam("/general/global_ref_frame", config_.global_frame) ||
           !pnh_.getParam("/general/robot_frame", config_.robot_frame) ||
           !pnh_.getParam("/general/tool_frame", config_.tool_frame))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(class_prefix_ + "Failed to get frame-data parameters (/general/..) from server");

            // Exit
            exit(-1);
        }

        // Report successful loaded parameters
        ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << ": Sucessfully loaded parameters from server"); 
    }

} // Namespace: Planner