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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_toolbox/tools/kinematics.h"

// Namespace: Robotics Toolbox
// -------------------------------
namespace Toolbox
{
// Kinematics Tool Class - Members:
// -------------------------------

    // Constants
    // -------------------------------
    const std::string Kinematics::class_prefix = "Toolbox::Kinematics::";


    // Get Current Joint-State
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentJointState(
            const std::string& topic,
            sensor_msgs::JointState& joint_state)
    {
        // Capture Current-Joint-State message from topic 
        sensor_msgs::JointStateConstPtr current_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(topic, ros::Duration(0.5));

        // Evaluate Current-Joint-State message
        if (!current_joint_state)
        {
            // Failed to capture Current-Joint-State message

            // Report to terminal
            ROS_ERROR_STREAM(class_prefix << __FUNCTION__ << 
                            " Failed to capture current joint-state message");

            // Function return
            return false;
        }

        // Check for empty joint-state position
        if(current_joint_state->position.empty())
        {
            // Empty Joint-State Position

            // Report to terminal
            ROS_ERROR_STREAM(class_prefix << __FUNCTION__ << 
                            " Failed: Current joint-state position is empty");

            // Function return
            return false;
        }

        // Update Joint-State to acquired current joint-state
        joint_state = *current_joint_state;

        // Function return
        return true;
    }


    // Get Current Joint-State
    // -------------------------------
    // (Function Overloading)
    bool Kinematics::getCurrentJointState(
            const std::string& topic,
            std::vector<double>& joint_position)
    {
        // Define local variable(s)
        sensor_msgs::JointState current_joint_state_;
        bool result_;

        // Get Current Joint-State
        if (Kinematics::getCurrentJointState(topic, current_joint_state_))
        {
            // Update Joint-State Position to acquired current joint-state posstion
            joint_position = current_joint_state_.position;

            // Function return
            return true;
        }
        
        // Get Current Joint-State failed
        return false;
    }

} // End Namespace: Robotics Toolbox