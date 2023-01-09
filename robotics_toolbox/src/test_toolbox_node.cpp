// Test Toolbox Node 
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

// Degree
void degreeRad()
{
    // Degree
    double deg = Toolbox::Common::radToDeg(M_PI/2);
    ROS_INFO_STREAM("Degree: " << deg);

    double rad = Toolbox::Common::degToRad(45.0);
    ROS_INFO_STREAM("Radians: " << rad);
}

// Colors
void colors()
{
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
}

// Axis type
void axistype()
{
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
}

// Trajectory
void traj()
{
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
}

// linspace
void linspace()
{
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
    ROS_INFO_STREAM(" ");
}

// rotation
void rotmat()
{
    //  Rotation Matrix
    // -------------------------------
    double x_rot = 15;
    double y_rot = -58;
    double z_rot = 37;

    // Quat
    Eigen::AngleAxisd pitch(Toolbox::Common::degToRad(x_rot), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd roll(Toolbox::Common::degToRad(y_rot), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(Toolbox::Common::degToRad(z_rot), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = pitch * roll * yaw;
    // Eigen::Quaternion<double> q = yaw * roll * pitch;
    Eigen::Matrix3d rot_quat = Toolbox::Math::rotMatQuat(q);

    ROS_INFO_STREAM(" Quaternion: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM(" Quat W: " << rot_quat(0));
    ROS_INFO_STREAM(" Quat X: " << rot_quat(1));
    ROS_INFO_STREAM(" Quat Y: " << rot_quat(2));
    ROS_INFO_STREAM(" Quat Z: " << rot_quat(3));
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_quat << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler XYZ
    Eigen::Matrix3d rot_xyz = Toolbox::Math::rotMatXYZ(x_rot, y_rot, z_rot);
    ROS_INFO_STREAM(" Rot XYZ: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_xyz << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler ZYX
    Eigen::Matrix3d rot_zyx = Toolbox::Math::rotMatZYX(z_rot, y_rot, x_rot);
    ROS_INFO_STREAM(" Rot ZYX: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_zyx << std::endl;
    ROS_INFO_STREAM(" ");

    // Euler ZYZ
    Eigen::Matrix3d rot_zyz = Toolbox::Math::rotMatZYZ(x_rot, y_rot, z_rot);
    ROS_INFO_STREAM(" Rot ZYZ: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Rotation Matrix: ");
    std::cout << rot_zyz << std::endl;
    ROS_INFO_STREAM(" ");
}

// transformation
void transformation()
{
    //  Rotation Matrix
    // -------------------------------
    double x_pos = 8.7;
    double y_pos = 9.6;
    double z_pos = -4.4;
    Eigen::Vector3d pos_vec(x_pos, y_pos, z_pos);
    
    double x_rot = 15;
    double y_rot = -58;
    double z_rot = 37;
    Eigen::Vector3d rot_vec(Toolbox::Common::degToRad(x_rot), Toolbox::Common::degToRad(y_rot), Toolbox::Common::degToRad(z_rot));
    Eigen::Matrix3d rot_xyz = Toolbox::Math::rotMatXYZ(rot_vec);

    // Quat
    Eigen::AngleAxisd pitch(Toolbox::Common::degToRad(x_rot), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd roll(Toolbox::Common::degToRad(y_rot), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(Toolbox::Common::degToRad(z_rot), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = pitch * roll * yaw;
    Eigen::Matrix3d rot_quat = Toolbox::Math::rotMatQuat(q);

    // Pos + Rot Mat
    Eigen::Isometry3d tm_rotmat = Toolbox::Math::transMat(pos_vec, rot_xyz);
    ROS_INFO_STREAM(" Transformation Matrix: Rotation Mat: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_rotmat.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_rotmat.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Pos + Quat
    Eigen::Isometry3d tm_quat = Toolbox::Math::transMat(pos_vec, rot_quat);
    ROS_INFO_STREAM(" Transformation Matrix: Quat Rot: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_quat.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_quat.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Pos + Rot Vec
    Eigen::Isometry3d tm_rotvec = Toolbox::Math::transMat(pos_vec, rot_vec);
    ROS_INFO_STREAM(" Transformation Matrix: Rot Vec: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_rotvec.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_rotvec.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

    // Scalar
    Eigen::Isometry3d tm_scalar = Toolbox::Math::transMat(x_pos, y_pos, z_pos, x_rot, y_rot, z_rot);
    ROS_INFO_STREAM(" Transformation Matrix: Scalar: ");
    ROS_INFO_STREAM(" ----------- ");
    ROS_INFO_STREAM("Translation: ");
    std::cout << tm_scalar.translation() << std::endl;
    ROS_INFO_STREAM("Rotation: ");
    std::cout << tm_scalar.rotation() << std::endl;
    ROS_INFO_STREAM(" ");

}

// Test Toolbox Node 
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
        
        // rotmat();
        transformation();


        // while (ros::ok())
        // {
        //     // TBD
        // }

    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}
