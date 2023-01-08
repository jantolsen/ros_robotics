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

    // Robotics Toolbox
    #include "robotics_toolbox/toolbox.h"
    #include "robotics_planner/planner_tool.h"

    // Messages
    #include "geometry_msgs/TwistStamped.h"

    // RVIZ
    #include <visualization_msgs/MarkerArray.h>

    // TF2
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    #include <tf2_ros/transform_listener.h>
    #include <tf2/convert.h>
    #include <tf2_eigen/tf2_eigen.h>
    #include <tf/tf.h>
    #include <tf_conversions/tf_eigen.h>

    // Eigen
    #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include <eigen_conversions/eigen_msg.h>

// Namespace: Trajectory
// -------------------------------
namespace Trajectory
{
    // Constants
    const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualization_trajectory";
    const std::string VISUALIZE_ARROW_TOPIC = "visualization_arrow";
    const std::string VISUALIZE_POSE_TOPIC = "visualization_pose";
    const std::string VISUALIZE_NORMPOSE_TOPIC = "visualization_norm";
    const std::string VISUALIZE_CSYS_TOPIC = "visualization_csys";

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

        // Test / Debug function
        void test();

        // Publish Pose Marker
        // -------------------------------
        /** \brief Publish a Pose-Marker to RVIZ
        * \param pose Pose of Marker [geometry_msgs::Pose] 
        */
        void publishMarkers(geometry_msgs::Pose pose);

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

        // ROS Subscriber(s)
        // -------------------------------
        ros::Subscriber test_sub_;       // Test subscriber

        // ROS Publisher(s)
        // -------------------------------
        ros::Publisher traj_marker_pub_;    // Trajectory visualization marker publisher (RVIZ)
        ros::Publisher arrow_marker_pub_;   // Arrow visualization marker publisher (RVIZ)
        ros::Publisher pose_marker_pub_;    // Pose visualization publisher (RVIZ)
        ros::Publisher norm_marker_pub_;    // Normal-Vector visualization publisher (RVIZ)
        ros::Publisher csys_marker_pub_;    // CSYS visualization publisher (RVIZ)

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