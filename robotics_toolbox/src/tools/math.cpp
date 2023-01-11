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
    std::vector<double> Math::linspace(
        double p_start, 
        double p_end, 
        int n)
    {
        // Define linear spaced vector
        std::vector<double> linspaced;

        // Check number of points
        if (n == 0)
        {
            // Empty linspace

            // Function return
            return linspaced;
        }
        // Only one point
        else if (n == 1)
        {
            // Assign only end-point
            linspaced.push_back(p_end);

            // Function return
            return linspaced;
        }

        // Calculate delta-spacer
        double delta = (p_end - p_start) / (n - 1);
        
        // Generate linear space
        for (int i = 0; i < (n-1); i++)
        {
            // Assign current point to linspace vector
            linspaced.push_back(p_start + delta*i);
        }
        
        // Assign last element of linspace vector to equal end-point
        linspaced.push_back(p_end);

        // Function return
        return linspaced;
    }

    // Linear Spaced Vector (Eigen::Vector3d)
    // -------------------------------
    // (Function overloading)
    std::vector<Eigen::Vector3d> Math::linspace(
        Eigen::Vector3d p_start, 
        Eigen::Vector3d p_end, 
        int n)
    {
        // Define linear spaced vector and local variables
        std::vector<Eigen::Vector3d> linspaced;
        std::vector<double> x, y, z;    

        // Generate linear space for each element of the Eigen::Vector3d
        // -------------------------------
        x = linspace(p_start[0], p_end[0], n);
        y = linspace(p_start[1], p_end[1], n);
        z = linspace(p_start[2], p_end[2], n);

        // Iterate over the number of points
        for (int i = 0; i < n; i++)
        {
            // Assign current element values to a point eigen vector
            Eigen::Vector3d point(x[i], y[i], z[i]);

            // Appned current point to linspace vector
            linspaced.push_back(point);
        }

        // Function return
        return linspaced;
    }

    // Interpolated Spaced Vector (double)
    // -------------------------------
    // (Function overloading)
    std::vector<double> Math::interpolateLinear(
        double p_start, 
        double p_end, 
        double dt)
    {
        // Define interval spaced vector
        std::vector<double> interpolated;

        // Check for negative or zero interval step
        if (dt <= 0)
        {
            // Empty Interpolation

            // Function return
            return interpolated;
        }
        
        // Calculate total number of step based on distance and delta
        double distance = p_end - p_start; 
        double steps = std::floor(distance/dt);

        // Linear interpolation
        for (int i = 0; i < steps; i++)
        {
            // Calculate delta-spacer
            double delta = i / steps;

            // Interpolate point
            double p = p_start + distance * delta;

            // Assign current point to interpolate vector
            interpolated.push_back(p);
        }

        // Assign last element of linspace vector to equal end-point
        interpolated.push_back(p_end);

        // Function return
        return interpolated;
    }

    // Interpolated Spaced Vector (Eigen::Vector3d)
    // -------------------------------
    // (Function overloading)
    std::vector<Eigen::Vector3d> Math::interpolateLinear(
        Eigen::Vector3d p_start, 
        Eigen::Vector3d p_end, 
        double dt)
    {
        // Define interval spaced vector
        std::vector<Eigen::Vector3d> interpolated;

        // Check for negative or zero interval step
        if (dt <= 0)
        {
            // Empty Interpolation

            // Function return
            return interpolated;
        }
        
        // Calculate totla number of step based on distance and delta
        Eigen::Vector3d distance = p_end - p_start; 
        double steps = std::floor(distance.norm() / dt);

        // Linear interpolation
        for (int i = 0; i < steps; i++)
        {
            // Calculate delta-spacer
            double delta = i / steps;

            // Interpolate point
            Eigen::Vector3d p = p_start + distance * delta;

            // Assign current point to interpolate vector
            interpolated.push_back(p);
        }

        // Assign last element of interpolation vector to equal end-point
        interpolated.push_back(p_end);

        // Function return
        return interpolated;
    }

    // Linear Segment with Parabolic Blends 
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Math::lspb(
        double p_start, 
        double p_end, 
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
                pos_vec.push_back(p_end);

                // Function return
                return pos_vec;
            }

            // Identical start- and end-point
            if(p_start == p_end)
            {
                // Assign values to vectors
                pos_vec = std::vector<double>(n, p_start);  // Fill with start-point values
                vel_vec = std::vector<double>(n, 0.0);      // Fill with zeros
                acc_vec = std::vector<double>(n, 0.0);      // Fill with zeros

                // Function return
                return pos_vec;
            }

        // Calculation
        // -------------------------------
            // Compute time-vector 
            // (using linspace to get evenly spaced vector with n-points) 
            std::vector<double> t = linspace(0.0, (n-1), n);

            // Get final time 
            // (equal to last element of time-vector)
            double tf = t.back();

            // Compute velocity
            double v = ((p_end - p_start) / tf) * 1.5;

            // Compute blending time
            double tb = (p_start - p_end + (v * tf)) / v;

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
                    pos = p_start + ((alpha / 2) * pow(td, 2));         // quadratic polynomial
                    vel = alpha * td;                                   // linear ramp 
                    acc = alpha;                                        // constant acceleration
                }
                // Linear motion
                else if (td <= (tf - tb))
                {
                    // Calculate trajectory components
                    pos = (p_end + p_start - (v * tf)) / 2 + (v * td);  // linear position
                    vel = v;                                            // constant velocity 
                    acc = 0;                                            // zero acceleration
                }
                // Final blending motion
                else
                {
                    pos = p_end - ((alpha / 2) * pow(tf, 2)) + (alpha * tf * td) - ((alpha / 2) * pow(td, 2));    // quadratic polynomial
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
    std::vector<Eigen::Vector3d> Math::lspb(
        Eigen::Vector3d p_start, 
        Eigen::Vector3d p_end, 
        int n)
    {
        // Define LSPB vector and local variables
        std::vector<Eigen::Vector3d> lspb_vec;
        std::vector<double> x, y, z;    

        // Generate LSPB for each element of the Eigen::Vector3d
        // -------------------------------
        x = lspb(p_start[0], p_end[0], n);
        y = lspb(p_start[1], p_end[1], n);
        z = lspb(p_start[2], p_end[2], n);

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

    // Linear Interpolation
    // -------------------------------
    // (Function Overloading)
    std::vector<double> Math::lerp(
        double p_start, 
        double p_end, 
        int n)
    {
        // Define linear spaced vector and local variables
        std::vector<double> interpolation;

        // Compute time interval-vector
        // (interval vector is a closed unit interval [0,1]
        // using linspace to get evenly spaced vector with n-points) 
        std::vector<double> t = linspace(0.0, 1.0, n);

        // Iterate over interval-vector
        for (int i = 0; t.size() < n; i++)
        {
            // Get timestep
            double td = t[i];

            // Calculate interpolation point
            double point = ((1 - td) * p_start) + (td * p_end);

            // Appned interpolation point to vector
            interpolation.push_back(point);
        }

        // Function return
        return interpolation;
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

    // Rotation Matrix - Quaternion
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatQuat(Eigen::Quaternion<double> quat)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        
        // Calculate Rotation Matrix
        m_rot = quat.toRotationMatrix();

        // Function return
        return m_rot;
    }

    // Rotation Matrix - Quaternion
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatQuat(double w, double x, double y, double z)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        Eigen::Quaternion<double> quat(w, x, y, z);
        
        // Calculate Rotation Matrix
        m_rot = quat.matrix();

        // Function return
        return m_rot;
    }

    // Rotation Matrix - XYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatXYZ(Eigen::Vector3d euler)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        
        // Calculate Rotations
        Eigen::AngleAxisd rot_x(euler(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_y(euler(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(euler(2), Eigen::Vector3d::UnitZ());

        // Calcuate Rotation Matrix
        m_rot = rot_x * rot_y * rot_z;

        // Function return
        return m_rot;
    }

    // Rotation Matrix - XYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatXYZ(double rot_x, double rot_y, double rot_z)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        Eigen::Vector3d euler(Common::degToRad(rot_x), 
                              Common::degToRad(rot_y), 
                              Common::degToRad(rot_z));
        
        // Calculate Rotation Matrix
        m_rot = rotMatXYZ(euler);

        // Function return
        return m_rot;
    }

    // Rotation Matrix - ZYX
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYX(Eigen::Vector3d euler)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        
        // Calculate Rotations
        Eigen::AngleAxisd rot_x(euler(0), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rot_y(euler(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z(euler(2), Eigen::Vector3d::UnitX());

        // Calcuate Rotation Matrix
        m_rot = rot_z * rot_y * rot_x;

        // Function return
        return m_rot;
    }

    // Rotation Matrix - ZYX
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYX(double rot_z, double rot_y, double rot_x)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        Eigen::Vector3d euler(Common::degToRad(rot_z), 
                              Common::degToRad(rot_y), 
                              Common::degToRad(rot_x));
        
        // Calculate Rotation Matrix
        m_rot = rotMatZYX(euler);

        // Function return
        return m_rot;
    }

    // Rotation Matrix - ZYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYZ(Eigen::Vector3d euler)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        
        // Calculate Rotations
        Eigen::AngleAxisd rot_z1(euler(0), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rot_y(euler(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_z2(euler(2), Eigen::Vector3d::UnitZ());

        // Calcuate Rotation Matrix
        m_rot = rot_z1 * rot_y * rot_z2;

        // Function return
        return m_rot;
    }

    // Rotation Matrix - XYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Matrix3d Math::rotMatZYZ(double rot_z1, double rot_y, double rot_z2)
    {
        // Define local variables
        Eigen::Matrix3d m_rot;
        Eigen::Vector3d euler(Common::degToRad(rot_z1), 
                              Common::degToRad(rot_y), 
                              Common::degToRad(rot_z2));
        
        // Calculate Rotation Matrix
        m_rot = rotMatZYZ(euler);

        // Function return
        return m_rot;
    }

    // Transformation Matrix
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(Eigen::Vector3d pos_vec, Eigen::Matrix3d rot_mat)
    {
        // Define local variables
        Eigen::Isometry3d tm;

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec; // Translation
        tm.linear() = rot_mat;      // Rotation

        // Function return
        return tm;
    }

    // Transformation Matrix
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(Eigen::Vector3d pos_vec, Eigen::Quaternion<double> quat)
    {
        // Define local variables
        Eigen::Isometry3d tm;

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec;     // Translation
        tm.linear() = quat.matrix();    // Rotation

        // Function return
        return tm;
    }

    // Transformation Matrix - Euler XYZ
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(
        Eigen::Vector3d pos_vec, 
        Eigen::Vector3d rot_vec, 
        int euler_seq)
    {
        // Define local variables
        Eigen::Isometry3d tm;
        Eigen::Matrix3d rot_mat;

        // Determine Euler-Sequence and find rotation matrix
        switch (euler_seq)
        {
        case XYZ:
            // Calculate Rotation-Matrix
            rot_mat = rotMatXYZ(rot_vec);
            break;

        case ZYX:
            // Calculate Rotation-Matrix
            rot_mat = rotMatZYX(rot_vec);
            break;
            
        case ZYZ:
            // Calculate Rotation-Matrix
            rot_mat = rotMatZYZ(rot_vec);
            break;

        default:
            // Calculate Rotation-Matrix
            rot_mat = rotMatXYZ(rot_vec);
            break;
        }

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec; // Translation
        tm.linear() = rot_mat;      // Rotation

        // Function return
        return tm;
    }

    // Transformation Matrix - Euler
    // -------------------------------
    // (Function Overloading)
    Eigen::Isometry3d Math::transMat(double pos_x, double pos_y, double pos_z,
                                     double rot_x, double rot_y, double rot_z,
                                     int euler_seq)
    {
        // Define local variables
        Eigen::Isometry3d tm;
        Eigen::Matrix3d rot_mat;
        Eigen::Vector3d pos_vec(pos_x, pos_y, pos_z);
        Eigen::Vector3d rot_vec;
        
        // Determine Euler-Sequence and find rotation matrix
        switch (euler_seq)
        {
        case XYZ:
            // Assign Rotation-Vector
            rot_vec = Eigen::Vector3d(Common::degToRad(rot_x), 
                                      Common::degToRad(rot_y), 
                                      Common::degToRad(rot_z));

            // Calculate Rotation-Matrix
            rot_mat = rotMatXYZ(rot_vec);
            break;

        case ZYX:
            // Assign Rotation-Vector
            rot_vec = Eigen::Vector3d(Common::degToRad(rot_z), 
                                      Common::degToRad(rot_y), 
                                      Common::degToRad(rot_x));

            // Calculate Rotation-Matrix
            rot_mat = rotMatZYX(rot_vec);
            break;
            
        case ZYZ:
            // Assign Rotation-Vector
            rot_vec = Eigen::Vector3d(Common::degToRad(rot_x), 
                                      Common::degToRad(rot_y), 
                                      Common::degToRad(rot_z));
                                      
            // Calculate Rotation-Matrix
            rot_mat = rotMatZYZ(rot_vec);
            break;

        default:
            // Report to terminal
            ROS_INFO_STREAM("Toolbox::Math::transMat: Assuming rot_z1 = rot_z and rot_z2 = rot_x");

            // Assign Rotation-Vector
            rot_vec = Eigen::Vector3d(Common::degToRad(rot_z), 
                                      Common::degToRad(rot_y), 
                                      Common::degToRad(rot_x));

            // Calculate Rotation-Matrix
            rot_mat = rotMatXYZ(rot_vec);
            break;
        }

        // Calculate Transformation Matrix        
        tm.translation() = pos_vec; // Translation
        tm.linear() = rot_mat;      // Rotation

        // Function return
        return tm;
        
                             
    }

} // End Namespace: Robotics Toolbox