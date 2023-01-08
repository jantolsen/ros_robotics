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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_toolbox/tools/math.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Math Tool Class - Members:
// -------------------------------

    // Linear Spaced Vector (double)
    // -------------------------------
    // (Function overloading)
    std::vector<double> Math::linspace(double p_start, double p_end, int n)
    {
        // Define linear spaced vector
        std::vector<double> linspaced;

        // Check number of points
        if (n = 0)
        {
            // Empty linspace

            // Function return
            return linspaced;
        }
        //
        else if (n = 1)
        {
            // Assign only end-point
            linspaced.push_back(p_end);

            // Function return
            return linspaced;
        }

        // Calculate delta-spacer
        double delta = (p_end - p_start) / (n - 1);

        // Generate linear space
        for (int i = 0; i < n; i++)
        {
            // Assign current point to linspace vector
            linspaced.push_back(p_start + delta*i);
        }
        
        // Assign last element of linspace vector to equal end-point
        linspaced.push_back(p_end);

        // Function return
        return linspaced;
    }

    // Linear Spaced Vector (int)
    // -------------------------------
    // (Function overloading)
    std::vector<double> Math::linspace(int p_start, int p_end, int n)
    {
        // Define linear spaced vector
        std::vector<double> linspaced;

        // Call Linspace (double)
        linspaced = linspace(double(p_start), double(p_end), n);

        // Function return
        return linspaced;
    }

    // Get Normal Vector (Transformation Matrix)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Math::getNormalVector(
        Eigen::Isometry3d tm,
        AxisType axis_type)
    {
        // Local variable holder(s)
        Eigen::Vector3d normal_vector;  // Normal-Vector of specifed axis (function return)

        // Calculate Axis-Type Normal-Vector represented as relative to given Transformation Matrix
        normal_vector = tm.rotation() * axis_type.unit_vec;

        // Function return
        return normal_vector;

    }   

    // Get Normal Vector (Pose)
    // -------------------------------
    // (Function Overloading)
    Eigen::Vector3d Math::getNormalVector(
        geometry_msgs::Pose pose,
        AxisType axis_type)
    {
        // Local variable holder(s)
        Eigen::Vector3d normal_vector;  // Normal-Vector of specifed axis (function return)
        Eigen::Isometry3d tm;           // Isometry Transformation Matrix

        // Get Transformation Matrix of Pose
        tf2::fromMsg(pose, tm);

        // Calculate Axis-Type Normal-Vector represented as relative to given Transformation Pose
        normal_vector = tm.rotation() * axis_type.unit_vec;

        // Function return
        return normal_vector;
    }


} // End Namespace: Robotics Toolbox