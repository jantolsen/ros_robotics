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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_toolbox/tools/trajectory.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Trajectory Tool Class - Members:
// -------------------------------

    // Generate Circular Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::genTrajCircular(
        Eigen::Vector3d center,
        double radius,
        double angle,
        int steps)
    {
        // REV1
        // -------------------------------
            //Local variables
            Eigen::Isometry3d tm;                   // Transformation Matrix
            std::vector<Eigen::Isometry3d> traj;    // Trajectory Transformations Matrices
            std::vector<double> thetas;             // Circle angle
            std::vector<double> phis;               // Free Axis rotation angle

            // Initialize
            thetas = Math::linspace(0.0, 2*M_PI, steps);    // Circular steps
            phis = Math::linspace(0.0, 2*M_PI, steps);      // Free Axis rotation
            traj.reserve(steps);                            // Reserve transformation matrices
            tm = Eigen::Isometry3d::Identity();             // Initialize transformation as identity matrix
            
            
            // Iterate over each circle step
            for (size_t i = 0; i < steps; i++)
            {
                // Current circle step and free axis rotation angle
                double theta = thetas[i];
                double phi = phis[i];
                
                // Translation
                Eigen::Vector3d pos;
                pos(0) = center(0) + radius * cos(theta);   // X-Position
                pos(1) = center(1) + radius * sin(theta);   // Y-Position
                pos(2) = center(2);                         // Z-Position

                // Add Position-vector as translation to transformation matrix
                tm.translation() = pos;

                // Orientation
                Eigen::AngleAxisd rot_z1(theta, Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd rot_y(angle, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd rot_z2(phi, Eigen::Vector3d::UnitZ());
                
                // Rotate transformation matrix
                tm.linear() = (rot_z1 * rot_y * rot_z2).matrix();

                // Append current transformation matrix
                traj.push_back(tm);
            }

        // Function return
        return traj;
    }
    
} // End Namespace: Robotics Toolbox