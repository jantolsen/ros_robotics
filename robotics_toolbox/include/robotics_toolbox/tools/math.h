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
        * Vector starts at p_start and ends at p_end 
        * with a total of n number of points   
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param n Total number of steps [int]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<double> linspace(
            double p_start, 
            double p_end, 
            int n);

        // Linear Spaced Vector 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a linear spaced vector
        * Vector starts at p_start and ends at p_end 
        * with a total of n number of points
        * \param p_start Start point [Eigen::Vector3d]
        * \param p_end End point [Eigen::Vector3d]
        * \param n Total number of steps [int]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<Eigen::Vector3d> linspace(
            Eigen::Vector3d p_start, 
            Eigen::Vector3d p_end, 
            int n);

        // Linear Interpolated Spaced Vector 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a step spaced vector
        * Starting at p1 and ending at p2 with stepping at dt intervals  
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param dt Interval step [double]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<double> interpolateLinear(
            double p_start, 
            double p_end, 
            double dt);

        // Linear Interpolated Spaced Vector 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Generate a step spaced vector
        * Starting at p1 and ending at p2 with stepping at dt intervals  
        * \param p_start Start point [Eigen::Vector3d]
        * \param p_end End point [Eigen::Vector3d]
        * \param dt Interval step [Eigen::Vector3d]
        * \return Linear spaced vector [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Vector3d> interpolateLinear(
            Eigen::Vector3d p_start, 
            Eigen::Vector3d p_end, 
            double dt);

        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create a Linear Segment with Parabolic Blends vector
        * Vector starts at p_start and ends at p_end 
        * with a total of n number of points 
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile. 
        * The trajectory consists of 3 parts:
        * t0 -> tb: Quadritc polynomial giving a linear ramped velocity
        * tb: Blending time, linear part with constant velocity
        * tb -> tf: Quadritc polynomial with linear ramp-down velocity
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param n Total number of steps [int]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<double> lspb(
            double p_start, 
            double p_end, 
            int n);

        // Linear Segment with Parabolic Blends 
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create a Linear Segment with Parabolic Blends vector
        * Vector starts at p_start and ends at p_end 
        * with a total of n number of points 
        * This type of trajectory has a trapezodial velocity profile,
        * which is appropriate when constant velocity is desired along 
        * a portion of the path, resulting in a parabolic position profile. 
        * The trajectory consists of 3 parts:
        * t0 -> tb: Quadritc polynomial giving a linear ramped velocity
        * tb: Blending time, linear part with constant velocity
        * tb -> tf: Quadritc polynomial with linear ramp-down velocity
        * \param p_start Start point [Eigen::Vector3d]
        * \param p_end End point [Eigen::Vector3d]
        * \param n Total number of steps [int]
        * \return Linear spaced vector [std::vector<double>]
        */
        static std::vector<Eigen::Vector3d> lspb(
            Eigen::Vector3d p_start, 
            Eigen::Vector3d p_end, 
            int n);

        // Linear Interpolation
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create Linear interpolated trajectory vector
        * Vector starts at p_start and ends at p_end
        * with a total of n number of points 
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param n Total number of steps [int]
        * \return Linear interpolated vector [std::vector<double>]
        */
        static std::vector<double> lerp(
            double p_start, 
            double p_end, 
            int n);

        // Linear Interpolation
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create Linear interpolated trajectory vector
        * Vector starts at p_start and ends at p_end
        * with a total of n number of points 
        * \param p_start Start point [double]
        * \param p_end End point [double]
        * \param n Total number of steps [int]
        * \return Linear interpolated vector [std::vector<double>]
        */
        static std::vector<Eigen::Vector3d> lerp(
            Eigen::Vector3d p_start, 
            Eigen::Vector3d p_end, 
            int n);

        // Spherical Linear Interpolation (eigen::quaternions)
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create Spherical linear interpolated trajectory vector
        * between two rotation points (quaternions). 
        * Vector starts at q_start and ends at q_end
        * with a total of n number of points 
        * This type of trajectory has a constant-speed along a unit-radius 
        * great circle arc
        * \param q_start Start rotation [Eigen::Quaterniond]
        * \param q_end End rotation [Eigen::Quaterniond]
        * \param n Total number of steps [int]
        * \return Spherical linear interpolated vector [std::vector<Eigen::Quaterniond>]
        */
        static std::vector<Eigen::Quaterniond> slerp(
            Eigen::Quaterniond q_start, 
            Eigen::Quaterniond q_end, 
            int n);

        // Spherical Linear Interpolation (Eigen::vector3d)
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Create Spherical linear interpolated trajectory vector
        * between two rotation points (euler). 
        * Vector starts at r_start and ends at r_end
        * with a total of n number of points 
        * This type of trajectory has a constant-speed along a unit-radius 
        * great circle arc
        * \param r_start Start euler rotation [Eigen::Vector3d]
        * \param r_end End euler rotation [Eigen::Vector3d]
        * \param n Total number of steps [int]
        * \param euler_seq Euler-Sequence (XYZ = 1, ZYX = 2, ZYZ = 3) [int]
        * \return Spherical linear interpolated vector [std::vector<Eigen::Vector3d>]
        */
        static std::vector<Eigen::Vector3d> slerp(
            Eigen::Vector3d r_start, 
            Eigen::Vector3d r_end, 
            int n,
            int euler_seq = XYZ);

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
            AxisType axis_type = Common::AXIS_Z);

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

        // Rotation Matrix - Quaternion
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for given quaternion
        * \param quat orientation [Eigen::Quaterniond] (Orientation [W,X,Y,Z])
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatQuat(Eigen::Quaternion<double> quat);

        // Rotation Matrix - Quaternion
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for given quaternion
        * \param w Scalar Coefficient W [double]
        * \param x Scalar Coefficient X [double]
        * \param y Scalar Coefficient Y [double]
        * \param z Scalar Coefficient Z [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatQuat(double w, double x, double y, double z);

        

        

        // Rotation Matrix - Quaternion-Vector
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Compute Rotation Matrix for given quaternion
        * \param q orientation [Eigen::Quaterniond] (Orientation [W,X,Y,Z])
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMat(
            Eigen::Quaternion<double> q);

        // Rotation Matrix - Quaternion-Scalar
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Compute Rotation Matrix for given quaternion scalar value
        * \param w Scalar Coefficient W [double]
        * \param x Scalar Coefficient X [double]
        * \param y Scalar Coefficient Y [double]
        * \param z Scalar Coefficient Z [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMat(
            double w, 
            double x, 
            double y, 
            double z);

        // Rotation Matrix - Euler-Vector
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Compute Rotation Matrix for given euler angles (rad)
        * \param euler Euler-Rotation (rad) [Eigen::Vector3d]
        * \param seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMat(
            Eigen::Vector3d euler,
            int seq = XYZ);

        // Rotation Matrix - Euler-Scalar
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Compute Rotation Matrix for given euler scalar angles (deg)
        * \param phi    1st axis-rotation (deg) [double]
        * \param theta  2nd axis rotation (deg) [double]
        * \param psi    3rd axis rotation (deg) [double]
        * \param seq    Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMat(
            double phi,
            double theta,
            double psi,
            int seq = XYZ);

        // Rotation Matrix - XYZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (rad) in XYZ-sequence
        * \param euler Euler-XYZ rotation (rad) [Eigen::Vector3d]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatXYZ(
            Eigen::Vector3d euler);

        // Rotation Matrix - XYZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (deg) in XYZ-sequence
        * \param rot_x X-Axis rotation (deg) [double]
        * \param rot_y Y-Axis rotation (deg) [double]
        * \param rot_z Z-Axis rotation (deg) [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatXYZ(
            double rot_x, 
            double rot_y, 
            double rot_z);

        // Rotation Matrix - ZYX
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (rad) in ZYX-sequence
        * \param euler Euler-ZYX rotation (rad) [Eigen::Vector3d]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZYX(
            Eigen::Vector3d euler);

        // Rotation Matrix - ZYX
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (deg) in ZYX-sequence
        * \param rot_z Z-Axis rotation (deg) [double]
        * \param rot_y Y-Axis rotation (deg) [double]
        * \param rot_x X-Axis rotation (deg) [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZYX(
            double rot_z, 
            double rot_y, 
            double rot_x);

        // Rotation Matrix - ZXZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (rad) in ZYZ-sequence
        * \param euler Euler-XYZ rotation (rad) [Eigen::Vector3d]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZXZ(
            Eigen::Vector3d euler);

        // Rotation Matrix - ZXZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (deg) in ZYZ-sequence
        * \param rot_z1 X-Axis rotation (deg) [double]
        * \param rot_x  X-Axis rotation (deg) [double]
        * \param rot_z2 Z-Axis rotation (deg) [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZXZ(
            double rot_z1, 
            double rot_x, 
            double rot_z2);

        // Rotation Matrix - ZYZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (rad) in ZYZ-sequence
        * \param euler Euler-XYZ rotation (rad) [Eigen::Vector3d]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZYZ(
            Eigen::Vector3d euler);

        // Rotation Matrix - ZYZ
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Rotation Matrix for euler angles (deg) in ZYZ-sequence
        * \param rot_z1 X-Axis rotation (deg) [double]
        * \param rot_y  Y-Axis rotation (deg) [double]
        * \param rot_z2 Z-Axis rotation (deg) [double]
        * \return Rotation Matrix [Eigen::Matrix3d]
        */
        static Eigen::Matrix3d rotMatZYZ(
            double rot_z1, 
            double rot_y, 
            double rot_z2);

        // Transformation Matrix
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Transformation Matrix for given 
        * Position Vector and Rotation Matrix 
        * \param pos_vec Position Vector [Eigen::Vector3d]
        * \param rot_mat Rotation Matrix [Eigen::Matrix3d]
        * \return Transformation Matrix [Eigen::Isometry3d]
        */
        static Eigen::Isometry3d transMat(
            Eigen::Vector3d pos_vec, 
            Eigen::Matrix3d rot_mat);

        // Transformation Matrix
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Transformation Matrix for given 
        * Position Vector and Quaternion orientation
        * \param pos_vec Position Vector [Eigen::Vector3d]
        * \param quat Orientation [Eigen::Quaternion<double>] (Orientation [W,X,Y,Z])
        * \return Transformation Matrix [Eigen::Isometry3d]
        */
        static Eigen::Isometry3d transMat(
            Eigen::Vector3d pos_vec, 
            Eigen::Quaternion<double> quat);

        // Transformation Matrix - Euler
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Transformation Matrix for given 
        * Position Vector and Euler-Rotation Vector
        * \param pos_vec Position Vector [Eigen::Vector3d]
        * \param rot_vec Euler rotation (rad) [Eigen::Vector3d]
        * \param euler_seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Transformation Matrix [Eigen::Isometry3d]
        */
        static Eigen::Isometry3d transMat(
            Eigen::Vector3d pos_vec, 
            Eigen::Vector3d rot_vec, 
            int euler_seq = XYZ);

        // Transformation Matrix - Euler Scalar
        // -------------------------------
        // Function Overloading:
        //      Multiple definitions of a function allows 
        //      for different ways of calling the function
        /** \brief Calculate Transformation Matrix for given 
        * Position scalar and Euler-Rotation (XYZ) scalars
        * \param pos_x  X-Axis Position (deg) [double]
        * \param pos_y  Y-Axis Position (deg) [double]
        * \param pos_z  Z-Axis Position (deg) [double]
        * \param phi    1st axis-rotation (deg) [double]
        * \param theta  2nd axis rotation (deg) [double]
        * \param psi    3rd axis rotation (deg) [double]
        * \param euler_seq Euler-Sequence for rotation (XYZ = 0, ZYX = 1, ZXZ = 2, ZYZ = 3) [int]
        * \return Transformation Matrix [Eigen::Isometry3d]
        */
        static Eigen::Isometry3d transMat(
            double pos_x, 
            double pos_y, 
            double pos_z,
            double phi, 
            double theta, 
            double psi,
            int euler_seq = XYZ);

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