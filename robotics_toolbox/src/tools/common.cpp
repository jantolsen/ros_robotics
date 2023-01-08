// Robotics Toolbox - Common Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Common Tools contains helper and utility functions 
//      useful for Robotics applications
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_toolbox/tools/common.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Common Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
        // X-Axis
        const struct AxisType Common::AXIS_X = 
        {
            .id = AXIS_ID_X,
            .name = "x_axis", 
            .unit_vec = Eigen::Vector3d::UnitX()
        };

        // Y-Axis
        const struct AxisType Common::AXIS_Y = 
        {
            .id = AXIS_ID_Y,
            .name = "y_axis", 
            .unit_vec = Eigen::Vector3d::UnitY()
        };

        // Z-Axis
        const struct AxisType Common::AXIS_Z = 
        {
            .id = AXIS_ID_Z,
            .name = "z_axis", 
            .unit_vec = Eigen::Vector3d::UnitZ()
        };

    // Convert Degrees to Radians
    // -------------------------------
    double Common::degToRad(double deg)
    {
        // Convert deg to rad
        double rad = deg * M_PI / 180.0;

        // Function Output
        return rad;
    }

    // Convert Radians to Degrees
    // -------------------------------
    double Common::radToDeg(double rad)
    {
        // Convert deg to rad
        double deg = rad * 180.0 / M_PI;

        // Function Output
        return deg;
    }

    // Convert Pose to Transform
    // -------------------------------
    // (Function overloading)
    geometry_msgs::Transform Common::poseToTransform(
        geometry_msgs::Pose pose)
    {
        // Define Transform variable holder
        geometry_msgs::Transform transform;
        
        // Convert Pose to Transform
        // Translation
        transform.translation.x = pose.position.x;
        transform.translation.y = pose.position.y;
        transform.translation.z = pose.position.z;

        // Rotation
        transform.rotation = pose.orientation;
    
        // Function Return
        return transform;
    }

    // Convert Pose to Transform-Stamped 
    // -------------------------------
    // (Function overloading)
    geometry_msgs::TransformStamped Common::poseToTransform(
        geometry_msgs::Pose pose,
        std::string parent_frame,
        std::string child_frame)
    {
        // Define Transform-Stamped variable holder
        geometry_msgs::TransformStamped transform_stamped;
        
        // Transform-Stamped Header
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = parent_frame;
        transform_stamped.child_frame_id = child_frame;

        // Convert Pose to Transform
        transform_stamped.transform = poseToTransform(pose);

        // Function Return
        return transform_stamped;
    }

    // Convert Pose to Transform-Stamped 
    // -------------------------------
    // (Function overloading)
    geometry_msgs::TransformStamped Common::poseToTransform(
        geometry_msgs::PoseStamped pose_stamped)
    {
        // Define Transform-Stamped variable holder
        geometry_msgs::TransformStamped transform_stamped;
        
        // Transform-Stamped Header
        transform_stamped.header = pose_stamped.header;

        // Convert Pose to Transform
        transform_stamped.transform = poseToTransform(pose_stamped.pose);

        // Function Return
        return transform_stamped;
    }

    // Convert Transform to Pose 
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::Pose Common::transformToPose(
        geometry_msgs::Transform transform)
    {
        // Define Pose variable holder
        geometry_msgs::Pose pose;
        
        // Convert Pose to Transform
        // Translation
        pose.position.x = transform.translation.x; 
        pose.position.y = transform.translation.y; 
        pose.position.z = transform.translation.z; 

        // Orientation
        pose.orientation = transform.rotation;
    
        // Function Return
        return pose;
    }

    // Convert Transform to Pose-Stamped 
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::PoseStamped Common::transformToPose(
        geometry_msgs::Transform transform,
        std::string parent_frame)
    {
        // Define Pose-Stamped variable holder
        geometry_msgs::PoseStamped pose_stamped;
        
        // Pose-Stamped Header
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = parent_frame;

        // Convert Pose to Transform
        pose_stamped.pose = transformToPose(transform);

        // Function Return
        return pose_stamped;
    }

    // Convert Transform-Stamped to Pose-Stamped
    // -------------------------------
    // (Function Overloading)
    geometry_msgs::PoseStamped Common::transformToPose(
        geometry_msgs::TransformStamped transform_stamped)
    {
        // Define Pose-Stamped variable holder
        geometry_msgs::PoseStamped pose_stamped;
        
        // Transform-Stamped Header
        pose_stamped.header = transform_stamped.header;

        // Convert Pose to Transform
        pose_stamped.pose = transformToPose(transform_stamped.transform);

        // Function Return
        return pose_stamped;
    }
        
} // End Namespace: Robotics Toolbox