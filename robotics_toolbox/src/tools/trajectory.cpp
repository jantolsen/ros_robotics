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

    // Generate Linear Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::trajectoryLinear(
        Eigen::Isometry3d pose_start,
        Eigen::Isometry3d pose_end,
        double delta)
    {
        // Local variables
        std::vector<Eigen::Isometry3d> traj;    // Trajectory Transformations Matrices
        std::vector<double> traj_step;          // Traj steps

        // Initialize

        pose_start.rotation();

        Eigen::Quaterniond q;
        Eigen::Quaterniond q0;
        Eigen::Quaterniond q1;
        
        q = q1.slerp(0.5, q1);

        // traj_step = Math::linspace(0.0, 2*M_PI, steps);    // Trajectory steps
    }

    // Generate Circular Trajectory
    // -------------------------------
    std::vector<Eigen::Isometry3d> Trajectory::trajectoryCircular(
        Eigen::Vector3d center,
        double radius,
        double angle,
        int steps)
    {
        // Local variables
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
            Eigen::Vector3d pos_vec;
            pos_vec(0) = center(0) + radius * cos(theta);   // X-Position
            pos_vec(1) = center(1) + radius * sin(theta);   // Y-Position
            pos_vec(2) = center(2);                         // Z-Position

            Eigen::Vector3d rot_vec;
            rot_vec(0) = theta;     // Z1-Rotation
            rot_vec(1) = angle;     // Y-Rotation
            rot_vec(2) = phi;       // Z2-Position

            // Rotation Matrix
            Eigen::Matrix3d rot_mat = Math::rotMatZYZ(rot_vec);

            // Create Transformation Matrix
            tm.translation() = pos_vec; // Translation
            tm.linear() = rot_mat;      // Rotation

            // Append current transformation matrix
            traj.push_back(tm);
        }

        // Function return
        return traj;
    }
    
} // End Namespace: Robotics Toolbox