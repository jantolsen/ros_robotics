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