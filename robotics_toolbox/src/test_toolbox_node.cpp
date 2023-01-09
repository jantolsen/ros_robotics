// Test Tooblox Node 
// -------------------------------
// Description:
//      Test Toolbox Node
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robotics_toolbox/toolbox.h"

// Prefix Parameter Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "toolbox_node");   
    
    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Main Code    
    // -------------------------------
        // Degree
        double deg = Toolbox::Common::radToDeg(M_PI/2);
        ROS_INFO_STREAM("Degree: " << deg);

        // Color
        std_msgs::ColorRGBA test_color = Toolbox::Visual::COLOR_RED;

        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Color Test: ");
        ROS_INFO_STREAM(" Color R: " << test_color.r);
        ROS_INFO_STREAM(" Color G: " << test_color.g);
        ROS_INFO_STREAM(" Color B: " << test_color.b);
        ROS_INFO_STREAM(" Color A: " << test_color.a);

        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Color GREEN: ");
        ROS_INFO_STREAM(" Color R: " << Toolbox::Visual::COLOR_GREEN.r);
        ROS_INFO_STREAM(" Color G: " << Toolbox::Visual::COLOR_GREEN.g);
        ROS_INFO_STREAM(" Color B: " << Toolbox::Visual::COLOR_GREEN.b);
        ROS_INFO_STREAM(" Color A: " << Toolbox::Visual::COLOR_GREEN.a);
        ROS_INFO_STREAM(" ");

        // Axis
        Toolbox::AxisType a = Toolbox::Common::AXIS_X; 

        ROS_INFO_STREAM(" Axis X: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Id: "     << a.id);
        ROS_INFO_STREAM(" Name: "   << a.name);

        ROS_INFO_STREAM(" Axis Z: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Id: "     << Toolbox::Common::AXIS_Z.id);
        ROS_INFO_STREAM(" Name: "   << Toolbox::Common::AXIS_Z.name);
        ROS_INFO_STREAM(" ");

        // TEST POSE
        // -------------------------------
        geometry_msgs::PoseStamped test_pose;
        test_pose.header.frame_id = "world";
        test_pose.header.stamp = ros::Time::now();

        test_pose.pose.position.x = 3.0;
        test_pose.pose.position.y = 2.0;
        test_pose.pose.position.z = 1.0;

        // Quaternion orientation (tf2::Quaternion)
        tf2::Quaternion rpy2quaternion_;                    

        // Convert RPY-orientation to Quaternion-orientation            
        // rpy2quaternion_.setRPY(Toolbox::Common::degToRad(7.5),
        //                         Toolbox::Common::degToRad(-68.0),
        //                         Toolbox::Common::degToRad(35.0)); 

        rpy2quaternion_.setRPY(Toolbox::Common::degToRad(0.0),
                                Toolbox::Common::degToRad(0.0),
                                Toolbox::Common::degToRad(90.0)); 

        test_pose.pose.orientation = tf2::toMsg(rpy2quaternion_);

        Eigen::Isometry3d pose_tf;              // Pose Isometry Transformation Matrix
        Eigen::Vector3d normal_vec_a;           // Normal-Vector of specifed axis (function return)
        Eigen::Vector3d normal_vec_b;           // Normal-Vector of specifed axis (function return)
        Eigen::Vector3d normal_vec_c;           // Normal-Vector of specifed axis (function return)
        Eigen::Vector3d normal_vec_d;           // Normal-Vector of specifed axis (function return)
        tf2::fromMsg(test_pose.pose, pose_tf);  // Get Transformation Matrix of Pose-CSYS

        normal_vec_a = pose_tf.matrix().col(0).head<3>().normalized();
        normal_vec_b = pose_tf.rotation() * Eigen::Vector3d::UnitX();
        normal_vec_c = pose_tf * Eigen::Vector3d::UnitX();

        ROS_INFO_STREAM(" Normal Vector A: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" x: "      << normal_vec_a(0));
        ROS_INFO_STREAM(" y: "      << normal_vec_a(1));
        ROS_INFO_STREAM(" z: "      << normal_vec_a(2));
        ROS_INFO_STREAM(" ");

        ROS_INFO_STREAM(" Normal Vector B: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" x: "      << normal_vec_b(0));
        ROS_INFO_STREAM(" y: "      << normal_vec_b(1));
        ROS_INFO_STREAM(" z: "      << normal_vec_b(2));
        ROS_INFO_STREAM(" ");

        ROS_INFO_STREAM(" Normal Vector C: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" x: "      << normal_vec_c(0));
        ROS_INFO_STREAM(" y: "      << normal_vec_c(1));
        ROS_INFO_STREAM(" z: "      << normal_vec_c(2));
        ROS_INFO_STREAM(" ");

        Eigen::Isometry3d new_pose = pose_tf.pretranslate(normal_vec_b);

        normal_vec_d = pose_tf.translation();

        ROS_INFO_STREAM(" POSE: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" x: "      << test_pose.pose.position.x);
        ROS_INFO_STREAM(" y: "      << test_pose.pose.position.y);
        ROS_INFO_STREAM(" z: "      << test_pose.pose.position.z);
        ROS_INFO_STREAM(" ");

        ROS_INFO_STREAM(" Normal Vector D: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" x: "      << normal_vec_d(0));
        ROS_INFO_STREAM(" y: "      << normal_vec_d(1));
        ROS_INFO_STREAM(" z: "      << normal_vec_d(2));
        ROS_INFO_STREAM(" ");

        // while (ros::ok())
        // {
        //     // TBD
        // }

        //  TRAJECTORY
        // -------------------------------
        Eigen::Isometry3d tm;
        std::vector<Eigen::Isometry3d> traj;
        Eigen::Vector3d pos;
        geometry_msgs::Pose pose;
        pos(0) = 3.0;   // X-Position
        pos(1) = 3.0;   // Y-Position
        pos(2) = 1.0;   // Z-Position

        traj = Toolbox::Trajectory::genTrajCircular(pos,
                                                    1.0,
                                                    Toolbox::Common::degToRad(22.5),
                                                    4);
        

        ROS_INFO_STREAM(" Trajectory: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Size: "         << traj.size());
        ROS_INFO_STREAM(" ----------- ");
        for (size_t i = 0; i < traj.size(); i++)
        {
            tf::poseEigenToMsg(traj[i], pose);

            
            ROS_INFO_STREAM(" Point" << i);
            ROS_INFO_STREAM(" ----------- ");
            ROS_INFO_STREAM(" Pose: ");
            ROS_INFO_STREAM(" X: " << pose.position.x);
            ROS_INFO_STREAM(" Y: " << pose.position.y);
            ROS_INFO_STREAM(" Z: " << pose.position.z);
            ROS_INFO_STREAM(" ");
        }

        
        //  LINSPACE
        // -------------------------------
        std::vector<double> linspace;
        linspace = Toolbox::Math::linspace(0.0, 2*M_PI, 4);

        ROS_INFO_STREAM(" Linspace: ");
        ROS_INFO_STREAM(" ----------- ");
        ROS_INFO_STREAM(" Size: "         << linspace.size());
        ROS_INFO_STREAM(" Point 0: "      << linspace[0]);
        ROS_INFO_STREAM(" Point 1: "      << linspace[1]);
        ROS_INFO_STREAM(" Point 2: "      << linspace[2]);
        ROS_INFO_STREAM(" Point 3: "      << linspace[3]);
        // ROS_INFO_STREAM(" Point 4: "      << linspace[4]);
        ROS_INFO_STREAM(" ");


    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}
