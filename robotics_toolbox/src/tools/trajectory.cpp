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
        std::vector<double> t)
    {
        // Define local variables
        double pos;                     // Position
        double vel;                     // Velocity
        double acc;                     // Acceleration
        std::vector<double> lspb_traj;  // LSBP trajectory (position)
        std::vector<double> pos_traj;   // Position trajectory
        std::vector<double> vel_traj;   // Velocity trajectory
        std::vector<double> acc_traj;   // Acceleration trajectory
        
        // Illegal argument handling
        // -------------------------------
            // Check for empty time period
            if (t.empty())
            {
                // Empty LSPB

                // Function return
                return lspb_traj;
            }
            // Time period only contains one step
            else if (t.size() == 1)
            {
                // Assign only end-point
                lspb_traj.push_back(p_f);

                // Function return
                return lspb_traj;
            }
            
            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to trajectory
                lspb_traj = std::vector<double>(t.size(), p_s); // Fill with start-point values
                pos_traj = std::vector<double>(t.size(), p_s);  // Fill with start-point values
                vel_traj = std::vector<double>(t.size(), 0.0);  // Fill with zeros
                acc_traj = std::vector<double>(t.size(), 0.0);  // Fill with zeros

                // Function return
                return lspb_traj;
            }

        // Calculation
        // -------------------------------
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
                pos_traj.push_back(pos);
                vel_traj.push_back(vel);
                acc_traj.push_back(acc);

                // Append trajectory points (position) to LSPB-Trajectory
                lspb_traj.push_back(pos);
            }

        // Function return
        return lspb_traj;
    }

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::lspb(
        double p_s, 
        double p_f, 
        int n)
    {
        // Define LSPB trajectory and time-period
        std::vector<double> lspb_traj;  // LSBP trajectory (position)

        // Illegal argument handling
        // -------------------------------
            // Check total number of steps
            if (n == 0)
            {
                // Empty LSPB

                // Function return
                return lspb_traj;
            }
            // Only one step
            else if (n == 1)
            {
                // Assign only end-point
                lspb_traj.push_back(p_f);

                // Function return
                return lspb_traj;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Calculate Quintic-Polynomial
            lspb_traj = lspb(p_s, p_f, t);

        // Function return
        return lspb_traj;
    }

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Trajectory::lspb(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        Eigen::VectorXd t)
    {
        // Define LSPB trajectory and local variables
        std::vector<Eigen::Vector3d> lspb_traj;
        std::vector<double> x, y, z;    

        // Convert Timer-Vector Eigen::VectorXd to std::vector<double>
        std::vector<double> time;    // Define vector
        time.resize(t.size());       // Resize to allocate memory 
        Eigen::Map<Eigen::VectorXd>(time.data(), time.size()) = t;    // Convert from Eigen::VectorX to std::vector

        // Generate LSPB for each element of the Eigen::Vector3d
        // -------------------------------
        x = lspb(p_s[AXIS_ID_X], p_f[AXIS_ID_X], time);
        y = lspb(p_s[AXIS_ID_Y], p_f[AXIS_ID_Y], time);
        z = lspb(p_s[AXIS_ID_Z], p_f[AXIS_ID_Z], time);

        // Iterate over the number of points
        for (int i = 0; i < time.size(); i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appened current point to LSBP trajectory
            lspb_traj.push_back(point);
        }

        // Function return
        return lspb_traj;
    }

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Trajectory::lspb(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        int n)
    {
        // Define LSPB trajectory
        std::vector<Eigen::Vector3d> lspb_traj;

        // Compute time-vector 
        // (using linspace to get evenly spaced vector with n-points) 
        std::vector<double> t = Math::linspace(0.0, (n-1), n);

        // Calculate Quintic-Polynomial
        // (converting time-vector from std::Vector<> to Eigen::VectorX)
        lspb_traj = lspb(p_s, p_f, Eigen::Map<Eigen::VectorXd>(t.data(), t.size()));

        // Function return
        return lspb_traj;
    }
    
    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyQuintic(
        double p_s, 
        double p_f, 
        std::vector<double> t,
        double v_s, 
        double v_f)
    {
        // Define local variables
        std::vector<double> poly_traj;  // Polynomial trajectory (position)
        Eigen::VectorXd pos_traj;       // Position trajectory
        Eigen::VectorXd vel_traj;       // Velocity trajectory
        Eigen::VectorXd acc_traj;       // Acceleration trajectory

        // Illegal argument handling
        // -------------------------------
            // Check for empty time period
            if (t.empty())
            {
                // Empty LSPB

                // Function return
                return poly_traj;
            }
            // Time period only contains one step
            else if (t.size() == 1)
            {
                // Assign only end-point
                poly_traj.push_back(p_f);

                // Function return
                return poly_traj;
            }

            // Identical start- and end-point
            if(p_s == p_f)
            {
                // Assign values to trajectory
                poly_traj = std::vector<double>(t.size(), p_s);     // Fill with start-point values
                pos_traj = Eigen::VectorXd::Ones(t.size()) * p_f;   // Fill with start-point values
                vel_traj = Eigen::VectorXd::Zero(t.size());         // Fill with zeros
                acc_traj = Eigen::VectorXd::Zero(t.size());         // Fill with zeros
                
                // Function return
                return poly_traj;
            }

        // Calculation
        // -------------------------------
            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Define the polynomial as a system of linear equation expressed in matrix form (Ax = b)
            // Then use x = inv(A) * b to solve for the polynomial coefficients

            // Matrix-equation
            Eigen::MatrixXd m(6,6);
            m << 0,                 0,                  0,                  0,              0,          1,      // q-start equation (position)
                 pow(tf, 5),        pow(tf, 4),         pow(tf, 3),         pow(tf, 2),     tf,         1,      // q-final quation (position)
                 0,                 0,                  0,                  0,              1,          0,      // qd-start equation (velocity)
                 5 * pow(tf, 4),    4 * pow(tf, 3),     3 * pow(tf, 2),     2 *tf,          1,          0,      // qd-final equation (velocity)
                 0,                 0,                  0,                  2,              0,          0,      // qdd-start equation (acceleration)
                 20 * pow(tf, 3),   12 * pow(tf, 2),    6 * tf,             2,              0,          0;      // qdd-final equation (acceleration)
            
            // Initial- and final-values as equation solutions
            Eigen::VectorXd q(6);
            q << p_s, p_f, v_s, v_f, 0, 0;

            // Calculate Coefficients for position
            // (using x = inv(A) * b )
            Eigen::VectorXd c(6);       
            c = m.inverse() * q;    
            
            // Calculate Coefficients for velocity
            // (Multiplying derivative values with position-coefficients)  
            Eigen::VectorXd c_d(5);     
            c_d << (5 * c[0]),
                   (4 * c[1]),
                   (3 * c[2]),
                   (2 * c[3]), 
                   (1 * c[4]);

            // Coefficients for acceleration
            // (Multiplying derivative values with velocity-coefficients)
            Eigen::VectorXd c_dd(4);
            c_dd << (4 * c_d[0]),
                    (3 * c_d[1]),
                    (2 * c_d[2]),
                    (1 * c_d[3]);

            // Evaluate polynomials
            // (converting time-vector from std::Vector<> to Eigen::VectorX)
            pos_traj = Math::polyval(c, Eigen::Map<Eigen::VectorXd>(t.data(), t.size()));
            vel_traj = Math::polyval(c_d, Eigen::Map<Eigen::VectorXd>(t.data(), t.size()));
            acc_traj = Math::polyval(c_dd, Eigen::Map<Eigen::VectorXd>(t.data(), t.size()));

        // Resize polynomial trajectory to equal time-period size
        poly_traj.resize(t.size());

        // Convert Trajectory Eigen::VectorXd to std::vector<double>
        Eigen::Map<Eigen::VectorXd>(poly_traj.data(), poly_traj.size()) = pos_traj;

        // Function return
        return poly_traj;
    }

    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Trajectory::polyQuintic(
        double p_s, 
        double p_f, 
        int n,
        double v_s, 
        double v_f)
    {
        // Define polynomial trajectory
        std::vector<double> poly_traj;   

        // Illegal argument handling
        // -------------------------------
            // Check total number of steps
            if (n == 0)
            {
                // Empty LSPB

                // Function return
                return poly_traj;
            }
            // Only one step
            else if (n == 1)
            {
                // Assign only end-point
                poly_traj.push_back(p_f);

                // Function return
                return poly_traj;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = Math::linspace(0.0, (n-1), n);

            // Calculate Quintic-Polynomial
            poly_traj = polyQuintic(p_s, p_f, t, v_s, v_f);

        // Function return
        return poly_traj;
    }

    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Trajectory::polyQuintic(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        Eigen::VectorXd t)
    {
        // Define Polynomial trajectory and local variables
        std::vector<Eigen::Vector3d> poly_traj;
        std::vector<double> x, y, z;    

        // Convert Timer-Vector Eigen::VectorXd to std::vector<double>
        std::vector<double> time;    // Define vector
        time.resize(t.size());       // Resize to allocate memory 
        Eigen::Map<Eigen::VectorXd>(time.data(), time.size()) = t;    // Convert from Eigen::VectorX to std::vector

        // Generate LSPB for each element of the Eigen::Vector3d
        // -------------------------------
        x = polyQuintic(p_s[AXIS_ID_X], p_f[AXIS_ID_X], time);
        y = polyQuintic(p_s[AXIS_ID_Y], p_f[AXIS_ID_Y], time);
        z = polyQuintic(p_s[AXIS_ID_Z], p_f[AXIS_ID_Z], time);

        // Iterate over the number of points
        for (int i = 0; i < time.size(); i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appened current point to LSBP trajectory
            poly_traj.push_back(point);
        }

        // Function return
        return poly_traj;
    }

    // Quintic Polynomial Trajectory
    // -------------------------------
    // (Function Overloading)
    std::vector<Eigen::Vector3d> Trajectory::polyQuintic(
        Eigen::Vector3d p_s, 
        Eigen::Vector3d p_f, 
        int n)
    {
        // Define LSPB trajectory
        std::vector<Eigen::Vector3d> poly_traj;

        // Compute time-vector 
        // (using linspace to get evenly spaced vector with n-points) 
        std::vector<double> t = Math::linspace(0.0, (n-1), n);

        // Calculate Quintic-Polynomial
        // (converting time-vector from std::Vector<> to Eigen::VectorX)
        poly_traj = polyQuintic(p_s, p_f, Eigen::Map<Eigen::VectorXd>(t.data(), t.size()));

        // Function return
        return poly_traj;
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