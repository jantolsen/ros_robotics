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

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef VISUAL_TOOL_H       
#define VISUAL_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // RVIZ Visualization
    #include <visualization_msgs/MarkerArray.h>
    #include <std_msgs/ColorRGBA.h>

    // Geometry
    #include "geometry_msgs/PoseStamped.h"

    // TF2
    #include <tf2_eigen/tf2_eigen.h>
    // #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    // #include <tf2_ros/transform_listener.h>
    // #include <tf2/convert.h>
    
    // #include <tf/tf.h>
    // #include <tf_conversions/tf_eigen.h>

    // Eigen
    // #include <Eigen/Core>
    #include <Eigen/Geometry>
    #include <eigen_conversions/eigen_msg.h>

    // Robotics Toolbox
    #include "robotics_toolbox/tools/common.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Visualization Tool Class
// -------------------------------
class Visual
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Set Color Red
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Red RGBA-Color
        * \param color Pointer to Color [std_msgs::ColorRGBA]
        */
        static void setColorRed(std_msgs::ColorRGBA& color);

        // Set Color Red
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Red RGBA-Color
        * \return Red Color [std_msgs::ColorRGBA]
        */
        static std_msgs::ColorRGBA setColorRed();

        // Set Color Blue
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Blue RGBA-Color
        * \param color Pointer to Color [std_msgs::ColorRGBA]
        */
        static void setColorBlue(std_msgs::ColorRGBA& color);

        // Set Color Blue
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Blue RGBA-Color
        * \return Red Color [std_msgs::ColorRGBA]
        */
        static std_msgs::ColorRGBA setColorBlue();

        // Set Color Green
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Green RGBA-Color
        * \param color Pointer to Color [std_msgs::ColorRGBA]
        */
        static void setColorGreen(std_msgs::ColorRGBA& color);

        // Set Color Green
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Assign Green RGBA-Color
        * \return Red Color [std_msgs::ColorRGBA]
        */
        static std_msgs::ColorRGBA setColorGreen();

        // Visualize Pose
        // -------------------------------
        /** \brief Create an Arrow Marker of a Pose
        * represented as a Marker to be publish to RVIZ
        * \param pose Pose [geometry_msgs::PoseStamped]
        * \param ns Namespace for Marker [std::string]
        * \param axis_type Axis-Type of Pose for which to display Marker Arrow [Toolbox::Axis] 
        * \param color Color of Pose Arrow [std_msgs::ColorRGBA] 
        * \param scale Scale of Pose Arrow [double] 
        * \return Arrow Marker [visualization_msgs::Marker]  
        */
        static visualization_msgs::Marker visualPose(
            geometry_msgs::PoseStamped pose,
            std::string ns,
            AxisType axis_type = Common::AXIS_X,
            std_msgs::ColorRGBA color = COLOR_RED,
            double scale = 0.25);

        // Visualize Pose Coordinate System
        // -------------------------------
        /** \brief Create a Coordinate System Marker of a Pose
        * represented as a Marker-Array to be publish to RVIZ
        * \param pose_csys Pose of CSYS [geometry_msgs::PoseStamped] 
        * \param scale Scale of CSYS arrows [double] 
        * \return Marker-Array of Pose CSYS [visualization_msgs::MarkerArray]  
        */
        static visualization_msgs::MarkerArray visualPoseCsys(
            geometry_msgs::PoseStamped pose,
            double scale = 0.25);

        // Constants
        // -------------------------------
        static const std_msgs::ColorRGBA COLOR_RED;
        static const std_msgs::ColorRGBA COLOR_BLUE;
        static const std_msgs::ColorRGBA COLOR_GREEN;

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
};
} // End Namespace: Robotics Toolbox
#endif // VISUAL_TOOL_H 