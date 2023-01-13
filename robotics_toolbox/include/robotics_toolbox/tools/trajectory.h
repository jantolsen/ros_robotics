// Robotics Toolbox - Trajectory Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Trajectory Tools contains helper and utility functions 
//      related to trajectory generation
//
// Version:
//  0.1 - Initial Version
//        [09.01.2023]  -   Jan T. Olsen
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
#ifndef TRAJECTORY_TOOL_H       
#define TRAJECTORY_TOOL_H

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
    #include "robotics_toolbox/tools/math.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Trajectory Tool Class
// -------------------------------
class Trajectory
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [double]
        * \param p_f    Trajectory finish point [double]
        * \param n      Trajectory total number of steps [int]
        * \return       Trajectory [std::vector<double>]
        */
        static std::vector<double> lspb(
            double p_s, 
            double p_f, 
            int n); 

        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a Linear Segment with Parabolic Blends trajectory
        * also known as a Trapozodial Trajectory
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile.
        * Trajectory starts at p_s and ends at p_f with a total of n number points
        * The trajectory consists of 3 parts:
        * ts -> tb: Linear ramped velocity, giving a quadratic polynomial motion
        * tb:       Blending time, with a constant velocity giving a linear motion
        * tb -> tf: Linear ramped down velocity, giving a quadratic polynomial motion
        * \param p_s    Trajectory start point [Eigen::Vector3d]
        * \param p_f    Trajectory finish point [Eigen::Vector3d]
        * \param n      Trajectory total number of steps [int]
        * \return       Trajectory [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Vector3d> lspb(
            Eigen::Vector3d p_s, 
            Eigen::Vector3d p_f, 
            int n);


        // Generate Linear Trajectory
        // -------------------------------
        /** \brief Generate Linear Trajectory
        * \param pose_start Start-Pose [Eigen::Isometry3d]
        * \param pose_end End-Pose [Eigen::Isometry3d]
        * \param delta Distance step between each pose in trajectory [double]
        * \return Trajectory [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Isometry3d> trajectoryLinear(
            Eigen::Isometry3d pose_start,
            Eigen::Isometry3d pose_end,
            double delta);

        // Generate Circular Trajectory
        // -------------------------------
        /** \brief Generate Circular Trajectory
        * \param center Circle center point [Eigen::Vector3d]
        * \param radius Circle radius [double]
        * \param angle Normal vector angle (rad) [double]
        * \param steps Resolution number of steps for trajectory [int]
        * \return Trajectory [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Isometry3d> trajectoryCircular(
            Eigen::Vector3d center,
            double radius,
            double angle,
            int steps);

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
#endif // TRAJECTORY_TOOL_H 