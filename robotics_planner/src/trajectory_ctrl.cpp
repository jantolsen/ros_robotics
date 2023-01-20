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
        norm_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
                            VISUALIZE_NORMPOSE_TOPIC,           // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        // Initialize publisher for csys visualization marker
        csys_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                            VISUALIZE_CSYS_TOPIC,           // Topic name
                            QUEUE_LENGTH);                  // Queue size 

        traj2_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                            "visualization_lineartrajectory",     // Topic name
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
        // visualization_msgs::MarkerArray csys_msg = Toolbox::Visual::visualPoseCsys(test_pose);
        // csys_marker_pub_.publish(csys_msg);


        // TRAJ CIRCULAR PUB
        std::vector<Eigen::Isometry3d> trajectory_tm;
        std::vector<geometry_msgs::PoseStamped> trajectory_pose; 
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.frame_id = "world";
        temp_pose.header.stamp = ros::Time::now();

        Eigen::Vector3d circle_position;
        circle_position(0) = 0.0;   // X-Position
        circle_position(1) = 3.0;   // Y-Position
        circle_position(2) = 1.0;   // Z-Position
        double circle_radius = 1.0;
        double offset_angle = Toolbox::Common::degToRad(45.0);
        int resolution = 100;

        trajectory_tm = Toolbox::Trajectory::trajectoryCircular(circle_position,
                                                            circle_radius,
                                                            offset_angle,
                                                            resolution);

        for (size_t i = 0; i < trajectory_tm.size(); i++)
        {
            // Convert Eigen Transformation to Pose
            tf::poseEigenToMsg(trajectory_tm[i], temp_pose.pose);

            // Append Pose to Trajectory Pose
            trajectory_pose.push_back(temp_pose);
        }
        
        visualization_msgs::MarkerArray traj_msg = Toolbox::Visual::visualPoseTrajectory(trajectory_tm, 0.5);
        csys_marker_pub_.publish(traj_msg);

        // NORMAL VEC
        visualization_msgs::Marker norm_arrow = Toolbox::Visual::visualNormalVector(test_pose.pose,
                                                                                    "norm_arrow");

        norm_marker_pub_.publish(norm_arrow);    



        // TRAJ Linear PUB

            // Transformation Start
            // -------------------------------
                // Translation
                double x0 = -2;
                double y0 = 0;
                double z0 = 1;
                Eigen::Vector3d pos0(x0, y0, z0);

                // Rotation
                double phi0 = 0;
                double theta0 = 0;
                double psi0 = 0;
                Eigen::Vector3d rot0(Toolbox::Common::degToRad(phi0), 
                                    Toolbox::Common::degToRad(theta0), 
                                    Toolbox::Common::degToRad(psi0));

                // Transformation
                Eigen::Isometry3d tm0 = Toolbox::Math::transMat(pos0, rot0);
                
            // Transformation Start
            // -------------------------------
                // Translation
                double x1 = -3;
                double y1 = 3;
                double z1 = 3;
                Eigen::Vector3d pos1(x1, y1, z1);

                // Rotation
                double phi1 = 0;
                double theta1 = 0;
                double psi1 = 90;
                Eigen::Vector3d rot1(Toolbox::Common::degToRad(phi1), 
                                    Toolbox::Common::degToRad(theta1), 
                                    Toolbox::Common::degToRad(psi1));

                // Transformation
                Eigen::Isometry3d tm1 = Toolbox::Math::transMat(pos1, rot1);

            // Trajectory
            // -------------------------------
                int steps = 5;
                std::vector<Eigen::Isometry3d> trajectory_linear = Toolbox::Trajectory::trajectoryLinear(tm0, tm1, steps);
            
        visualization_msgs::MarkerArray traj_lin_msg = Toolbox::Visual::visualPoseTrajectory(trajectory_linear, 0.5);
        traj_marker_pub_.publish(traj_lin_msg);
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