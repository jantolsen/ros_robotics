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

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::lspb(
        double p_s, 
        double p_f, 
        int n)
    {
        // Define local variables
        double pos;                     // Position
        double vel;                     // Velocity
        double acc;                     // Acceleration
        std::vector<double> pos_vec;    // Position vector
        std::vector<double> vel_vec;    // Velocity vector
        std::vector<double> acc_vec;    // Acceleration vector

        // Illegal argument handling
        // -------------------------------
            // Check total number of steps
            if (n == 0)
            {
                // Empty LSPB

                // Function return
                return pos_vec;
            }
            // Only one step
            else if (n == 1)
            {
                // Assign only end-point
                pos_vec.push_back(p_f);

                // Function return
                return pos_vec;
            }

            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to vectors
                pos_vec = std::vector<double>(n, p_s);  // Fill with start-point values
                vel_vec = std::vector<double>(n, 0.0);  // Fill with zeros
                acc_vec = std::vector<double>(n, 0.0);  // Fill with zeros

                // Function return
                return pos_vec;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Compute velocity
            double v = ((p_f - p_s) / tf) * 1.5;

            // Compute blending time
            double tb = (p_s - p_f + (v * tf)) / v;

            // Compute alpha
            // (Helper variable)
            double alpha = v / tb;

            // Iterate over the time-vector
            // (calculate trajectory components)
            for (int i = 0; i < t.size(); i++)
            {
                // Get timestep
                double td = t[i];

                // Initial blending motion
                if (td <= tb)
                {
                    // Calculate trajectory components
                    pos = p_s + ((alpha / 2) * pow(td, 2)); // quadratic polynomial
                    vel = alpha * td;                       // linear ramp 
                    acc = alpha;                            // constant acceleration
                }
                // Linear motion
                else if (td <= (tf - tb))
                {
                    // Calculate trajectory components
                    pos = (p_f + p_s - (v * tf)) / 2 + (v * td);    // linear position
                    vel = v;                                        // constant velocity 
                    acc = 0;                                        // zero acceleration
                }
                // Final blending motion
                else
                {
                    pos = p_f - ((alpha / 2) * pow(tf, 2)) + (alpha * tf * td) - ((alpha / 2) * pow(td, 2));    // quadratic polynomial
                    vel = (alpha * tf) - (alpha * td);                  // linear ramp 
                    acc = -alpha;                                       // constant acceleration
                }
                
                // Append trajectory components to respective vectors
                pos_vec.push_back(pos);
                vel_vec.push_back(vel);
                acc_vec.push_back(acc);
            }

        // Function return
        return pos_vec;
    }

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Trajectory::lspb(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        int n)
    {
        // Define LSPB vector and local variables
        std::vector<Eigen::Vector3d> lspb_vec;
        std::vector<double> x, y, z;    

        // Generate LSPB for each element of the Eigen::Vector3d
        // -------------------------------
        x = lspb(p_s[AXIS_ID_X], p_f[AXIS_ID_X], n);
        y = lspb(p_s[AXIS_ID_Y], p_f[AXIS_ID_Y], n);
        z = lspb(p_s[AXIS_ID_Z], p_f[AXIS_ID_Z], n);

        // Iterate over the number of points
        for (int i = 0; i < n; i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appned current point to linspace vector
            lspb_vec.push_back(point);
        }

        // Function return
        return lspb_vec;
    }

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