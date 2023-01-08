// Robotics Toolbox - Math Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Math Tools contains helper and utility functions 
//      related to mathematical calculations
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
#ifndef MATH_TOOL_H       
#define MATH_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robotics_toolbox/tools/common.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Math Tool Class
// -------------------------------
class Math
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Linear Spaced Vector 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a linear spaced vector
        * Starting at p1 and ending at p2 with n points  
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param n Number of points [int]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<double> linspace(double p_start, double p_end, int n);

        // Linear Spaced Vector 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a linear spaced vector
        * Starting at p1 and ending at p2 with n points  
        * \param p_start Start point [int]
        * \param p_end End point [int]
        * \param n Number of points [int]
        * \return Linear spaced vector [std::vector<int>]
        */
        static std::vector<double> linspace(int p_start, int p_end, int n);

        // Get Normal Vector (Transformation Matrix)
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Get the Normal-Vector of a specific axis (X, Y, or Z) 
        * for the given Transformation Matrix   
        * \param tm Transformation Matrix [Eigen::Isometry3d]
        * \param axis_type Axis for Normal-Vector calculation [Toolbox::Axis]
        * \return Normal-Vector [Eigen::Vector3d]
        */
        static Eigen::Vector3d getNormalVector(
            Eigen::Isometry3d tm,
            AxisType axis_type = Common::AXIS_Z;

        // Get Normal Vector (Pose)
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Get the Normal-Vector of a specific axis (X, Y, or Z) 
        * for the given Pose 
        * \param pose Pose [geometry_msgs::Pose] (Position: [X,Y,Z] & Orientation [X,Y,Z,W])
        * \param axis_type Axis for Normal-Vector calculation [Toolbox::Axis] 
        * \return Normal-Vector [Eigen::Vector3d]
        */
        static Eigen::Vector3d getNormalVector(
            geometry_msgs::Pose pose,
            AxisType axis_type = Common::AXIS_Z);

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
#endif // MATH_TOOL_H