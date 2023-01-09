// Robotics Toolbox - Visualization Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Visualization Tools contains helper and utility functions 
//      related to display of objects in RVIZ
//
// Version:
//  0.1 - Initial Version
//        [06.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_toolbox/tools/visual.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Visualization Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std_msgs::ColorRGBA Visual::COLOR_RED = setColorRed();
    const std_msgs::ColorRGBA Visual::COLOR_BLUE = setColorBlue();
    const std_msgs::ColorRGBA Visual::COLOR_GREEN = setColorGreen();
    const std_msgs::ColorRGBA Visual::COLOR_YELLOW = setColorYellow();

    // Set Color Red (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorRed(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }

    // Set Color Red (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorRed()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Red
        setColorRed(color);

        // Function return:
        return color;
    }

    // Set Color Blue (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorBlue(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }

    // Set Color Blue (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorBlue()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Blue
        setColorBlue(color);

        // Function return:
        return color;
    }

    // Set Color Green (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorGreen(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }

    // Set Color Green (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorGreen()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Green
        setColorGreen(color);

        // Function return:
        return color;
    }

    // Set Color Yellow (Color-Pointer)
    // -------------------------------
    // (Function overloading)
    void Visual::setColorYellow(std_msgs::ColorRGBA& color)
    {
        // Assign RGBA-values
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }

    // Set Color Yellow (Return color)
    // -------------------------------
    // (Function overloading)
    std_msgs::ColorRGBA Visual::setColorYellow()
    {
        // Define Color
        std_msgs::ColorRGBA color;
        
        // Set Color Yellow
        setColorYellow(color);

        // Function return:
        return color;
    }
    
    // Visualize Pose
    // -------------------------------
    visualization_msgs::Marker Visual::visualPose(
            geometry_msgs::PoseStamped pose,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure arrow marker
        // -------------------------------
        marker.ns = ns;
        marker.id = axis_type.id;
        marker.color = color;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = pose.header.frame_id;
        marker.scale.x = scale / 15;    // arrow shaft diameter
        marker.scale.y = scale / 10;    // arrow head diameter
        marker.scale.z = scale / 5;     // arrow head length
        marker.points.reserve(2);       // reserve points for marker (start and end of arrow)

        // Create Axis Markers
        // -------------------------------
        Eigen::Isometry3d pose_tm;      // Pose Isometry Transformation Matrix
        geometry_msgs::Point p_start;   // Pose arrow start-point    
        geometry_msgs::Point p_end;     // Pose arrow end-point

        // Set marker start point
        p_start = pose.pose.position;

        // Get Transformation Matrix of Pose-CSYS
        tf2::fromMsg(pose.pose, pose_tm);

        // Calculate marker direction based on unit-vector and scale
        Eigen::Vector3d marker_dir = pose_tm * (axis_type.unit_vec * scale);
        
        // Calculate end point of axis
        tf::pointEigenToMsg(marker_dir, p_end);

        // Assign start- and end point to arrow marker
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // Function return
        return marker;
    }
        
    // Visualize Pose Coordinate System
    // -------------------------------
    visualization_msgs::MarkerArray Visual::visualPoseCsys(
        geometry_msgs::PoseStamped pose_csys, 
        double scale)
    {
        // Define rviz markers
        visualization_msgs::Marker x_axis, y_axis, z_axis;
        visualization_msgs::MarkerArray csys;

        // Create CSYS axis arrow markers
        // -------------------------------
            // X-Axis Arrow Marker
            x_axis = Toolbox::Visual::visualPose(pose_csys,                 // Pose of Coordinate system
                                                "csys/x_axis",              // Namespace for arrow marker
                                                Common::AXIS_X,             // Axis of axis-arrow-marker
                                                Visual::COLOR_RED,          // Color of axis-arrow marker 
                                                scale);                     // Scale of axis-arrow marker

            // Y-Axis Arrow Marker
            y_axis = Toolbox::Visual::visualPose(pose_csys,                 // Pose of Coordinate system
                                                "csys/y_axis",              // Namespace for arrow marker  
                                                Common::AXIS_Y,             // Axis of axis-arrow-marker 
                                                Visual::COLOR_GREEN,        // Color of axis-arrow marker   
                                                scale);                     // Scale of axis-arrow marker

            // Z-Axis Arrow Marker
            z_axis = Toolbox::Visual::visualPose(pose_csys,                 // Pose of Coordinate system
                                                "csys/z_axis",              // Namespace for arrow marker
                                                Common::AXIS_Z,             // Axis of axis-arrow-marker
                                                Visual::COLOR_BLUE,         // Color of axis-arrow marker
                                                scale);                     // Scale of axis-arrow marker

        // Assign Axis-Markers to CSYS Marker Array
        csys.markers.push_back(x_axis);
        csys.markers.push_back(y_axis);
        csys.markers.push_back(z_axis);

        // Function return
        return csys;
    }

    // Visualize Normal-Vector
    // -------------------------------
    visualization_msgs::Marker Visual::visualNormalVector(
            geometry_msgs::Pose pose,
            std::string ns,
            AxisType axis_type,
            std_msgs::ColorRGBA color,
            double scale)
    {
        // Define rviz marker
        visualization_msgs::Marker marker;

        // Configure arrow marker
        // -------------------------------
        marker.ns = ns;
        marker.id = axis_type.id;
        marker.color = color;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0);
        marker.header.frame_id = "world";
        marker.scale.x = scale / 15;    // arrow shaft diameter
        marker.scale.y = scale / 10;    // arrow head diameter
        marker.scale.z = scale / 5;     // arrow head length
        marker.points.reserve(2);       // reserve points for marker (start and end of arrow)

        // Local variable holders
        Eigen::Vector3d v_start;        // Start-Point vector  
        Eigen::Vector3d v_end;          // End-Point vector
        Eigen::Vector3d v_normal;       // Normal vector

        geometry_msgs::Point p_start;   // Pose arrow start-point    
        geometry_msgs::Point p_end;     // Pose arrow end-point

        // Calculation
        // -------------------------------
        // Convert pose start-point to Eigen Vector
        tf::pointMsgToEigen(pose.position, v_start);

        // Calculate Normal-Vector
        v_normal = Math::getNormalVector(pose, axis_type);

        // Calculate end-point vector
        v_end = v_start + v_normal*scale;

        // Create Arrow Marker
        // -------------------------------
        // Convert eigen vector to point
        tf::pointEigenToMsg(v_start, p_start);
        tf::pointEigenToMsg(v_end, p_end);

        // Assign start- and end point to arrow marker
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // Function return
        return marker;
    }

    // Visualize Pose Trajectory
    // -------------------------------
    visualization_msgs::MarkerArray Visual::visualPoseTrajectory(
        std::vector<geometry_msgs::PoseStamped> pose_trajectory,
        double scale)
    {
        // Define rviz markers
        visualization_msgs::Marker x_axis, y_axis, z_axis;
        visualization_msgs::MarkerArray trajectory;

        // Iterate over each Pose of trajectory vector
        for (size_t i = 0; i < pose_trajectory.size(); i++)
        {
            // Point Marker Namespace
            std::string point_name = "point_" + std::to_string(i); 

            // Create CSYS axis arrow markers
            // -------------------------------
            // X-Axis Arrow Marker
            x_axis = Toolbox::Visual::visualPose(pose_trajectory[i],        // Current Pose of Trajectory
                                                point_name + "/x_axis",    // Namespace for arrow marker
                                                Common::AXIS_X,             // Axis of axis-arrow-marker
                                                Visual::COLOR_RED,          // Color of axis-arrow marker 
                                                scale);                     // Scale of axis-arrow marker

            // Y-Axis Arrow Marker
            y_axis = Toolbox::Visual::visualPose(pose_trajectory[i],        // Current Pose of Trajectory
                                                point_name + "/x_axis",    // Namespace for arrow marker  
                                                Common::AXIS_Y,             // Axis of axis-arrow-marker 
                                                Visual::COLOR_GREEN,        // Color of axis-arrow marker   
                                                scale);                     // Scale of axis-arrow marker

            // Z-Axis Arrow Marker
            z_axis = Toolbox::Visual::visualPose(pose_trajectory[i],        // Current Pose of Trajectory
                                                point_name + "/x_axis",    // Namespace for arrow marker
                                                Common::AXIS_Z,             // Axis of axis-arrow-marker
                                                Visual::COLOR_BLUE,         // Color of axis-arrow marker
                                                scale);                     // Scale of axis-arrow marker

            // Assign Axis-Markers to Trajectory Marker Array
            trajectory.markers.push_back(x_axis);
            trajectory.markers.push_back(y_axis);
            trajectory.markers.push_back(z_axis);
        }

        // Function return
        return trajectory;
    }
    
} // End Namespace: Robotics Toolbox