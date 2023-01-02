// Joystick Control
// ----------------------------------------------
// Description:
//      Robotics Joystick Control
//      Subscribes to a joystick-node (joy_node (http://wiki.ros.org/joy) 
//      or spacenav_node (http://wiki.ros.org/spacenav_node) dependent on the active external controller)
//      Based on the incomming joystick command, the controller maps and publishes
//      corresponding cartesian velocities and joint velocities
//
//      Based on MoveIt-Servo's "spacenav_to_twist" 
//      https://github.com/ros-planning/moveit/tree/master/moveit_ros/moveit_servo
//
// Version:
//  0.1 - Initial Version
//        [21.12.2022]  -   Jan T. Olsen
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
#ifndef JOYCTRL_H       
#define JOYCTRL_H   

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <string>
    #include <map>
    
    // ROS
    #include <ros/ros.h>

    // Messages
    #include "geometry_msgs/TwistStamped.h"
    #include "control_msgs/JointJog.h"
    #include "sensor_msgs/Joy.h"

// Namespace: Joystick
// -------------------------------
namespace Joystick
{
    // Struct: Joy Topic-Config
    struct JoyTopicConfig
    {
        std::string joy_topic;      // Topic-Name for Joystick-Input subscriber
        std::string twist_topic;    // Topic-Name for Twist-Command publisher (cartesian)
        std::string joint_topic;    // Topic-Name for Joint-Command publisher 
    };

    // Struct: Joy Axis-Config
    struct JoyAxisConfig
    {
        std::string name;           // name of axis
        int index;                  // index number for axis of joystick
        double raw_value_min;       // raw minimum value for joystick input
        double raw_value_max;       // raw maximum value for joystick input
        double deadzone_value;      // deadzone value for joystick input
        double scale_value_min;     // scale minimum value for axis command
        double scale_value_max;     // scale maximum value for axis command
        double value = 0.0;         // axis value (default: 0.0)
    };

    // Struct: Joy Button-Config
    struct JoyButtonConfig
    {
        std::string name;       // name of button
        int index;              // index number for button of joystick
        bool value = false;     // button value (default: false)
        bool value_old = false; // old button value (default: false)
    };

    // Enum: Control Modes
    enum ControlMode
    {
        MODE_XYZ,               // Task-Space: [X, Y, Z]
        MODE_RPY,               // Task-Space: [Roll, Pitch, Yaw]
        MODE_XYZRPY,            // Task-Space: [X, Y, Z, Roll, Pitch, Yaw]
        MODE_JOINT_Q13,         // Joint-Space: [Joint 1, Joint 2, Joint 3] 
        MODE_JOINT_Q46,         // Joint-Space: [Joint 4, Joint 5, Joint 6] 
        MODE_JOINT_Q16,         // Joint-Space: [Joint 1, .. Joint 6]
    };

    // Enum: Task-Space
    enum TaskSpace
    {
        LINEAR_X,
        LINEAR_Y,
        LINEAR_Z,
        ANGULAR_X,
        ANGULAR_Y,
        ANGULAR_Z
    };

    // Enum: Joint-Space
    enum JointSpace
    {
        JOINT_1,
        JOINT_2,
        JOINT_3,
        JOINT_4,
        JOINT_5,
        JOINT_6
    };

    // Enum: Buttons
    enum Buttons
    {
        BTN_NEXT_MODE,
        BTN_PREV_MODE,
        BTN_SPARE_3,
        BTN_SPARE_4,
        BTN_SPARE_5,
        BTN_SPARE_6
    };

// Joystick Control Class
// -------------------------------
class JoyCtrl
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Class constructor
        // -------------------------------
        JoyCtrl(
            ros::NodeHandle& nh,
            ros::NodeHandle& pnh);

        // Class destructor
        // -------------------------------
        ~JoyCtrl();

    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Member Variables
        // -------------------------------
        ros::NodeHandle nh_;                // ROS Nodehandle
        ros::NodeHandle pnh_;               // ROS Private Nodehandle
        ros::AsyncSpinner asyncspinner_;    // ROS Asynchronous Spinner

        std::string msg_prefix_ = "JoyCtrl: ";          // Class message-prefix for termnial output
        std::vector<std::string> joint_names_;          // Robot Joint-Names
        JoyTopicConfig topic_config_;                   // Topic configuration
        std::map<int, JoyAxisConfig> taskspace_map_;    // Taskspace-Command map (cartesian)
        std::map<int, JoyAxisConfig> jointspace_map_;   // Jointspace-Command map
        std::map<int, JoyButtonConfig> button_map_;     // Button-Command map
        int control_mode_ = MODE_XYZ;                   // Joystick control mode XYZ [default], RPY, JOINT Q13, JOINT Q46)

        // ROS Subscriber(s)
        // -------------------------------
        ros::Subscriber joy_sub_;       // Subscriber to joystick-input

        // ROS Publisher(s)
        // -------------------------------
        ros::Publisher twist_pub_;      // Twist-Command publisher (cartesian)
        ros::Publisher joint_pub_;      // Joint-Command publisher        

        // Class initialiation
        // -------------------------------
        /** \brief Initialization of Joy-Ctrl class.
        * Called from class constructor
        */
        void init();

        // Get Robot Joint-Names
        // -------------------------------
        /** \brief Get the Robot Joint-Names from parameter server (loaded together with node)
        * \return Joint-Names [std::vector<std::string>]
        */
        std::vector<std::string> getRobotJointNames();

        // Get Joystick Topic Configuration
        // -------------------------------
        /** \brief Get the Topic-Configuration for Joystick-Control from parameter server (loaded together with node)
        * \return Joystick Topic Configuration [struct: JoyTopicConfig]
        */
        JoyTopicConfig getJoyTopicConfig();

        // Get Joystick Axis Configuration
        // -------------------------------
        /** \brief Get a Joystick Axis Configuration from the parameter server (loaded together with node)
        * Configuration contains Name, Axis-Index, Scaling-Factor, Deadzone-Factor, etc. 
        * for the related joystick controller.
        * \param axis_param Axis Configuration Parameters [XmlRpc::XmlRpcValue::ValueStruct] 
        * \return Joystick Axis Configuration [struct: JoyAxisConfig]
        */
        JoyAxisConfig getJoyAxisConfig(
            XmlRpc::XmlRpcValue xmlrpc_axis_param);

        // Get Joystick Button Configuration
        // -------------------------------
        /** \brief Get a Joystick Button Configuration from the parameter server (loaded together with node)
        * Configuration contains Name, Button-Index, etc. 
        * for the related joystick controller.
        * \param button_param Button Configuration Parameters [XmlRpc::XmlRpcValue::ValueStruct] 
        * \return Joystick Button Configuration [struct: JoyButtonConfig]
        */
        JoyButtonConfig getJoyButtonConfig(
            XmlRpc::XmlRpcValue xmlrpc_button_param);

        // Get Joystick Task-Space Configuration
        // -------------------------------
        /** \brief Get the Joystick Task-Space Configuration from the parameter server (loaded together with node)
        * This includes each the Axis-Configuration for each axis of the related joystick controller
        * \return Joystick Task-Space command map [std::map<int, JoyAxisConfig>]
        */
        std::map<int, JoyAxisConfig> getJoyTaskSpaceMap();

        // Get Joystick Joint-Space Configuration
        // -------------------------------
        /** \brief Get the Joystick Joint-Space Configuration from the parameter server (loaded together with node)
        * This includes each the Axis-Configuration for each axis of the related joystick controller
        * \return Joystick Joint-Space command map [std::map<int, JoyAxisConfig>]
        */
        std::map<int, JoyAxisConfig> getJoyJointSpaceMap();

        // Get Joystick Button-Map Configuration
        // -------------------------------
        /** \brief Get the Joystick Button-Map Configuration from the parameter server (loaded together with node)
        * This includes each the Button-Configuration for each Button of the related joystick controller
        * \return Joystick Button command map [std::map<int, JoyButtonConfig>]
        */
        std::map<int, JoyButtonConfig> getJoyButtonMap();

        // Joystick Callback-Function
        // -------------------------------
        /** \brief Callback function for joy-node subscriber 
        * handles data from incomming joystick input
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        */
        void joystickCB(
            const sensor_msgs::Joy::ConstPtr& msg);

        // Get Servo-Command Twist (Cartesian)
        // -------------------------------
        /** \brief Calculate Servo-Command Twist (Cartesian) using data from joystick input
        * and Task-Space map of joystick axis configuration 
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        * \param taskspace_map Joystick Task-Space command map [std::map<int, JoyAxisConfig>]
        * \param control_mode Joystick Twist Control Mode (XYZ, RPY, XYZRPY) [int]
        * \return Servo-Command Twist [geometry_msgs::TwistStamped]
        */
        geometry_msgs::TwistStamped getServoCommandTwist(
            const sensor_msgs::Joy::ConstPtr& msg,
            std::map<int, JoyAxisConfig>& taskspace_map,
            const int control_mode = MODE_XYZRPY);

        // Get Servo-Command Joint
        // -------------------------------
        /** \brief Calculate Servo-Command Joint using data from joystick input
        * and Task-Space map of joystick axis configuration 
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        * \param jointspace_map Joystick Joint-Space command map [std::map<int, JoyAxisConfig>]
        * \param control_mode Joystick Joint Control Mode (Q13, Q46, Q16) [int]
        * \return Servo-Command Joint [control_msgs::JointJog]
        */
        control_msgs::JointJog getServoCommandJoint(
            const sensor_msgs::Joy::ConstPtr& msg,
            std::map<int, JoyAxisConfig>& jointspace_map,
            const int control_mode = MODE_JOINT_Q16);

        // Get Joystick Control-Mode
        // -------------------------------
        /** \brief Determine Joystick Control-Mode from joystick
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        * \param button_map Joystick Joint-Space command map [std::map<int, JoyAxisConfig>]
        * \param control_mode Joystick Control-Mode [int]
        */
        void getJoyControlMode(
            const sensor_msgs::Joy::ConstPtr& msg,
            std::map<int, JoyButtonConfig>& button_map,
            int& control_mode);

        // Update Joystick Axis
        // -------------------------------
        /** \brief Update Joystick Axis with joystick input
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        * \param joy_axis Joystick Axis [struct: JoyAxisConfig]
        */
        void updateJoyAxis(
            const sensor_msgs::Joy::ConstPtr& msg,
            JoyAxisConfig& axis);

        // Update Joystick Button
        // -------------------------------
        /** \brief Update Joystick Button with joystick input
        * \param msg Joystick input data [sensor_msgs::Joy::ConstPtr]
        * \param joy_axis Joystick Button [struct: JoyButtonConfig]
        */
        void updateJoyButton(
            const sensor_msgs::Joy::ConstPtr& msg,
            JoyButtonConfig& button);

        // Rescale and Deadzone
        // -------------------------------
        /** \brief Rescale raw input value and compensate for deadzone.
        * \param raw_value Raw input value [double]
        * \param raw_min Raw minimum value [double]
        * \param raw_max Raw maximum value [double]
        * \param raw_deadzone Raw deadzone value [double] 
        * \param scale_min Scale minimum value [double]
        * \param scale_max Scale maximum value [double]
        * \return Scaled value [double]
        */
        double calcRescaleDeadzone(
            double raw_value,
            double raw_min,
            double raw_max,
            double raw_deadzone,
            double scale_min,
            double scale_max);

    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:

        // Constants
        static const int NUM_SPINNERS = 1;
        static const int QUEUE_LENGTH = 1;
        
        // Taskspace Emum Map
        const std::map<const int, const std::string> TASKSPACE_MAP = 
        {
            {LINEAR_X,  "linear-x" },
            {LINEAR_Y,  "linear-y" },
            {LINEAR_Z,  "linear-z" },
            {ANGULAR_X, "angular-x"},
            {ANGULAR_Y, "angular-y"},
            {ANGULAR_Z, "angular-z"}
        };

        // Jointspace Emum Map
        const std::map<const int, const std::string> JOINTSPACE_MAP = 
        {
            {JOINT_1,   "joint-1"},
            {JOINT_2,   "joint-2"},
            {JOINT_3,   "joint-3"},
            {JOINT_4,   "joint-4"},
            {JOINT_5,   "joint-5"},
            {JOINT_6,   "joint-6"}
        };

        // Control Mode Enum Map
        const std::map<const int, const std::string> CTRL_MODE_MAP = 
        {
            {MODE_XYZ,          "XYZ"      },
            {MODE_RPY,          "RPY"      },
            {MODE_XYZRPY,       "XYZ & RPY"},
            {MODE_JOINT_Q13,    "JOINT 1-3"},
            {MODE_JOINT_Q46,    "JOINT 4-6"},
            {MODE_JOINT_Q16,    "JOINT ALL"}
        };
};

}   // Namespace: Joystick
#endif //JOYCTRL_H