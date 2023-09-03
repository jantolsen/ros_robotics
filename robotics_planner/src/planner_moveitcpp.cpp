// Planner MoveIt-Cpp 
// -------------------------------
// Description:
//      
//      
//
// Version:
//  0.1 - Initial Version
//        [14.06.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_planner/planner_moveitcpp.h"

// Namespace(s)
// -------------------------------
namespace rvt = rviz_visual_tools;

    // Class constructor
    // -------------------------------
    PlannerMoveitCpp::PlannerMoveitCpp(
        ros::NodeHandle& nh)
    :
        nh(nh)
    {
        // Debug
        ROS_INFO("TEST MoveIt-Cpp");

        // Initialize and Execute
        exec();

    }

    // Class destructor
    // -------------------------------
    PlannerMoveitCpp::~PlannerMoveitCpp()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }

    // Class execution
    // -------------------------------
    void PlannerMoveitCpp::exec()
    {
        /* Otherwise robot with zeros joint_states */
        ros::Duration(1.0).sleep();

        auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh);
        moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

        auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
        auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
        auto robot_start_state = planning_components->getStartState();
        auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

        // Visualization
        // ^^^^^^^^^^^^^
        //
        // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
        // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
        moveit_visual_tools::MoveItVisualTools visual_tools("robot_tool0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                            moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "MoveItCpp Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools.trigger();

        // Start the demo
        // ^^^^^^^^^^^^^^^^^^^^^^^^^
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

        // // END_TUTORIAL
        // visual_tools.deleteAllMarkers();
        // visual_tools.prompt("Press 'next' to end the demo");
    }
