// Robotics Toolbox - Kinematics Tools
// -------------------------------
// Description:
//      Toolbox for Robotics
//      Kinematics Tools contains helper and utility functions 
//      related to kinematic calculations
//
// Version:
//  0.1 - Initial Version
//        [22.02.2023]  -   Jan T. Olsen
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
#ifndef KINEMATIC_TOOL_H       
#define KINEMATIC_TOOL_H

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

// Kinematic Tool Class
// -------------------------------
class Kinematics
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:


};  // End Class: Kinematics
} // End Namespace: Robotics Toolbox
#endif // MATH_TOOL_H