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
        
        // // Start Async-Spinner
        // asyncspinner_.start();
        // ros::waitForShutdown();
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
        // ROS Publisher(s)
        // -------------------------------
        // Initialize publisher for trajectory visualization marker
        traj_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                            VISUALIZE_TRAJECTORY_TOPIC,     // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Initialize publisher for arrow visualization marker
        arrow_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
                            VISUALIZE_ARROW_TOPIC,          // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Initialize publisher for pose visualization marker
        pose_marker_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                            VISUALIZE_POSE_TOPIC,           // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Initialize publisher for pose visualization marker
        norm_marker_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                            VISUALIZE_NORMPOSE_TOPIC,           // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Initialize publisher for csys visualization marker
        csys_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                            VISUALIZE_CSYS_TOPIC,           // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Test / Debug
        test();
        
    }

    // Publish Pose Marker
    // -------------------------------
    void TrajectoryControl::publishMarkers(geometry_msgs::Pose pose)
    {
        // TEST POSE
        // -------------------------------
        geometry_msgs::PoseStamped test_pose;
        test_pose.header.frame_id = "world";
        test_pose.header.stamp = ros::Time::now();

        test_pose.pose.position.x = 2.0;
        test_pose.pose.position.y = 2.0;
        test_pose.pose.position.z = 0.5;

        // Quaternion orientation (tf2::Quaternion)
        tf2::Quaternion rpy2quaternion_;                    

        // Convert RPY-orientation to Quaternion-orientation            
        rpy2quaternion_.setRPY(Toolbox::Common::degToRad(45.0),
                                0.0,
                                Toolbox::Common::degToRad(45.0)); 

        test_pose.pose.orientation = tf2::toMsg(rpy2quaternion_);
        
        // POSE PUB
        pose_marker_pub_.publish(test_pose);    
        
        // Arrow PUB
        visualization_msgs::Marker arrow = Toolbox::Visual::visualPose(test_pose,
                                                                        "pose_arrow",
                                                                        Toolbox::Common::AXIS_X,
                                                                        Toolbox::Visual::COLOR_GREEN,
                                                                        0.5);
        arrow_marker_pub_.publish(arrow);

        // CSYS PUB
        visualization_msgs::MarkerArray csys_msg = Toolbox::Visual::visualPoseCsys(test_pose);
        csys_marker_pub_.publish(csys_msg);


        // NORMAL VEC
        geometry_msgs::PoseStamped norm_pose = test_pose;
        geometry_msgs::Vector3 norm_vec;
        Eigen::Vector3d norm = Toolbox::Math::getNormalVector(test_pose.pose);

        // Calculate end point of axis
        // tf::poseEigenToMsg(norm, norm_pose.pose.position);
        tf::vectorEigenToMsg(norm, norm_vec);

        norm_pose.header.frame_id = "world";
        norm_pose.header.stamp = ros::Time::now();
        norm_pose.pose.position.x += norm_vec.x;
        norm_pose.pose.position.y += norm_vec.y;
        norm_pose.pose.position.z += norm_vec.z;

        norm_marker_pub_.publish(norm_pose);    
    }

    
    // Test / Debug function
    // -------------------------------
    void TrajectoryControl::test()
    {
        std_msgs::ColorRGBA red = Toolbox::Visual::COLOR_RED;

        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Color RED: ");
        ROS_INFO_STREAM(" Color R: " << red.r);
        ROS_INFO_STREAM(" Color G: " << red.g);
        ROS_INFO_STREAM(" Color B: " << red.b);
        ROS_INFO_STREAM(" Color A: " << red.a);

        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Color GREEN: ");
        ROS_INFO_STREAM(" Color R: " << Toolbox::Visual::COLOR_GREEN.r);
        ROS_INFO_STREAM(" Color G: " << Toolbox::Visual::COLOR_GREEN.g);
        ROS_INFO_STREAM(" Color B: " << Toolbox::Visual::COLOR_GREEN.b);
        ROS_INFO_STREAM(" Color A: " << Toolbox::Visual::COLOR_GREEN.a);


        geometry_msgs::Pose test;
        publishMarkers(test);

    }

} // Namespace: Trajectory