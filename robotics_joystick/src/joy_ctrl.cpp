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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_joystick/joy_ctrl.h"

// Namespace: Joystick
// -------------------------------
namespace Joystick
{

    // Class constructor
    // -------------------------------
    JoyCtrl::JoyCtrl(
        ros::NodeHandle& nh,
        ros::NodeHandle& pnh)
    :
        nh_(nh),
        pnh_(pnh),
        asyncspinner_(NUM_SPINNERS)
    {
        // Initialize
        init();
        
        // Start Async-Spinner
        asyncspinner_.start();
        ros::waitForShutdown();
    }

    // Class destructor
    // -------------------------------
    JoyCtrl::~JoyCtrl()
    {
        // Report to terminal
        ROS_INFO_STREAM(msg_prefix_ + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }

    // Class initialiation
    // -------------------------------
    void JoyCtrl::init()
    {
        // Get Robot Joint-Names
        joint_names_ = getRobotJointNames();

        // Get Joystick topic configuration
        topic_config_ = getJoyTopicConfig();

        // Get Joystick Task-Space configuration
        taskspace_map_ = getJoyTaskSpaceMap();

        // Get Joystick Joint-Space configuration
        jointspace_map_ = getJoyJointSpaceMap();

        // Get Joystick Button-Map configuration
        button_map_ = getJoyButtonMap();

        // ROS Subscriber(s)
        // -------------------------------
        // Initialize subscriber to joystick input data
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(topic_config_.joy_topic, // Topic name
                                                    QUEUE_LENGTH,           // Queue size
                                                    &JoyCtrl::joystickCB,   // Callback function
                                                    this);                  // Class Object

        // ROS Publisher(s)
        // -------------------------------
        // Initialize publisher for joystick cartesian command
        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_config_.twist_topic,  // Topic name
                                                                QUEUE_LENGTH);              // Queue size  

        // Initialize publisher for joystick joint command
        joint_pub_ = nh_.advertise<control_msgs::JointJog>(topic_config_.joint_topic,       // Topic name
                                                            QUEUE_LENGTH);                  // Queue size     

        // Debug
        // -------------------------------
        int i;
        for (i = LINEAR_X; i <= ANGULAR_Z; i++)
        {
            ROS_INFO_STREAM("-------------------------------");
            ROS_INFO_STREAM("AXIS Name: "      << taskspace_map_[i].name);
            ROS_INFO_STREAM("AXIS Index: "     << taskspace_map_[i].index);
            ROS_INFO_STREAM("AXIS Scale min: " << taskspace_map_[i].scale_value_min);
            ROS_INFO_STREAM("AXIS Scale max: " << taskspace_map_[i].scale_value_max);
            ROS_INFO_STREAM("AXIS Deadband: "  << taskspace_map_[i].deadzone_value);
            ROS_INFO_STREAM("AXIS Value: "     << taskspace_map_[i].value);
        }

        int j;
        for (j = JOINT_1; j <= JOINT_6; j++)
        {
            ROS_INFO_STREAM("-------------------------------");
            ROS_INFO_STREAM("JOINT Name: "      << jointspace_map_[j].name);
            ROS_INFO_STREAM("JOINT Index: "     << jointspace_map_[j].index);
            ROS_INFO_STREAM("JOINT Scale min: " << jointspace_map_[j].scale_value_min);
            ROS_INFO_STREAM("JOINT Scale max: " << jointspace_map_[j].scale_value_max);
            ROS_INFO_STREAM("JOINT Deadband: "  << jointspace_map_[j].deadzone_value);
            ROS_INFO_STREAM("JOINT Value: "     << jointspace_map_[j].value);
        }
        ROS_ERROR_STREAM("-------------------------------");

        int k;
        for (k = BTN_NEXT_MODE; k <= BTN_SPARE_6; k++)
        {
            ROS_INFO_STREAM("-------------------------------");
            ROS_INFO_STREAM("Button Name: "        << button_map_[k].name);
            ROS_INFO_STREAM("Button Index: "       << button_map_[k].index);
            ROS_INFO_STREAM("Button Value: "       << button_map_[k].value);
            ROS_INFO_STREAM("Button Old Value: "   << button_map_[k].value_old);
        }
        ROS_INFO_STREAM("-------------------------------");
    }

    // Get Robot Joint-Names
    // -------------------------------
    std::vector<std::string> JoyCtrl::getRobotJointNames()
    {
        // Defining temporary variable holder
        std::string robot_prefix;
        std::vector<std::string> joint_names;

        // Get Robot-Prefix parameter from Parameter Server
        if(!pnh_.getParam("robot_prefix", robot_prefix))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Robot-Prefix");
        }

        // Get Robot Joint-Names from Parameter Server
        if(!pnh_.getParam("/" + robot_prefix + "/controller_joint_names", joint_names))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Robot-Joint-Names for Robot-Prefix (" + robot_prefix.c_str() + ")");
        }

        // Function return
        return joint_names;
    }

    // Get Joystick Topic Configuration
    // -------------------------------
    JoyTopicConfig JoyCtrl::getJoyTopicConfig()
    {
        // Defining temporary variable holder
        JoyTopicConfig joy_topic_config;
        XmlRpc::XmlRpcValue xmlrpc_topicconfig;

        // Get Joystick topic parameter from Parameter Server
        if(!pnh_.getParam("joystick_config/topic", xmlrpc_topicconfig))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Joystick Topic Configuration");
        }

        // Assign topic parameters to topic-config struct
        joy_topic_config.joy_topic = (std::string)xmlrpc_topicconfig["joy_interface"];
        joy_topic_config.twist_topic = (std::string)xmlrpc_topicconfig["twist_interface"];
        joy_topic_config.joint_topic = (std::string)xmlrpc_topicconfig["joint_interface"];

        // Function return
        return joy_topic_config;
    }

    // Get Joystick Axis Configuration
    // -------------------------------
    JoyAxisConfig JoyCtrl::getJoyAxisConfig(
        XmlRpc::XmlRpcValue xmlrpc_axis_param)
    {
        // Defining temporary variable holder
        JoyAxisConfig joy_axis_config;

        // Assign topic parameters to topic-config struct
        joy_axis_config.name = (std::string)xmlrpc_axis_param["name"];
        joy_axis_config.index = xmlrpc_axis_param["index"];
        joy_axis_config.raw_value_min = xmlrpc_axis_param["raw_min"];
        joy_axis_config.raw_value_max = xmlrpc_axis_param["raw_max"];
        joy_axis_config.deadzone_value = xmlrpc_axis_param["deadband"];
        joy_axis_config.scale_value_min = xmlrpc_axis_param["scale_min"];
        joy_axis_config.scale_value_max = xmlrpc_axis_param["scale_max"];

        // Function return
        return joy_axis_config;
    }

    // Get Joystick Button Configuration
    // -------------------------------
    JoyButtonConfig JoyCtrl::getJoyButtonConfig(
        XmlRpc::XmlRpcValue xmlrpc_button_param)
    {
        // Defining temporary variable holder
        JoyButtonConfig joy_button_config;

        // Assign topic parameters to topic-config struct
        joy_button_config.name = (std::string)xmlrpc_button_param["name"];
        joy_button_config.index = xmlrpc_button_param["index"];

        // Function return
        return joy_button_config;
    }

    // Get Joystick Task-Space Configuration
    // -------------------------------
    std::map<int, JoyAxisConfig> JoyCtrl::getJoyTaskSpaceMap()
    {
        // Defining temporary variable holder
        std::map<int, JoyAxisConfig> taskspace_joymap;
        XmlRpc::XmlRpcValue xmlrpc_taskspace_config;
        int enum_iterator = 0;

        // Get Joystick Task-Space Configuration parameter from Parameter Server
        if(!pnh_.getParam("joystick_config/taskspace_config", xmlrpc_taskspace_config))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Joystick Task-Space Configuration");
        }

        // Iterate over each Axis-Configuration
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xmlrpc_taskspace_config.begin(); it != xmlrpc_taskspace_config.end(); it++) 
        {
            // Get Axis Configuration for current element of the Task-Space configuration
            JoyAxisConfig axis_config = getJoyAxisConfig(xmlrpc_taskspace_config[it->first]);

            // Assign Axis Configuration with related identifier to Task-Space Joy-Map
            taskspace_joymap.insert(
            {
                TaskSpace(enum_iterator),
                axis_config
            });

            // Increment enum-iterator
            enum_iterator ++;
        }

        // Function return
        return taskspace_joymap;
    }

    // Get Joystick Joint-Space Configuration
    // -------------------------------
    std::map<int, JoyAxisConfig> JoyCtrl::getJoyJointSpaceMap()
    {
        // Defining temporary variable holder
        std::map<int, JoyAxisConfig> jointspace_joymap;
        XmlRpc::XmlRpcValue xmlrpc_jointspace_config;
        int enum_iterator = 0;

        // Get Joystick Joint-Space Configuration parameter from Parameter Server
        if(!pnh_.getParam("joystick_config/jointspace_config", xmlrpc_jointspace_config))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Joystick Joint-Space Configuration");

        }

        // Iterate over each Axis-Configuration
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xmlrpc_jointspace_config.begin(); it != xmlrpc_jointspace_config.end(); it++) 
        {
            // Get Axis Configuration for current element of the Joint-Space configuration
            JoyAxisConfig axis_config = getJoyAxisConfig(xmlrpc_jointspace_config[it->first]);

            // Assign Axis Configuration with related identifier to Joint-Space Joy-Map
            jointspace_joymap.insert(
            {
                JointSpace(enum_iterator),
                axis_config
            });

            // Increment enum-iterator
            enum_iterator ++;
        }

        // Function return
        return jointspace_joymap;
    }

    // Get Joystick Button-Map Configuration
    // -------------------------------
    std::map<int, JoyButtonConfig> JoyCtrl::getJoyButtonMap()
    {
        // Defining temporary variable holder
        std::map<int, JoyButtonConfig> button_joymap;
        XmlRpc::XmlRpcValue xmlrpc_button_config;
        int enum_iterator = 0;

        // Get Joystick Button Configuration parameter from Parameter Server
        if(!pnh_.getParam("joystick_config/button_config", xmlrpc_button_config))
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Failed to get Joystick Button Configuration");

        }

        // Iterate over each Button-Configuration
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xmlrpc_button_config.begin(); it != xmlrpc_button_config.end(); it++) 
        {
            // Get Button Configuration for current element of the Button-Map configuration
            JoyButtonConfig button_config = getJoyButtonConfig(xmlrpc_button_config[it->first]);

            // Assign Button Configuration with related identifier to Button Joy-Map
            button_joymap.insert(
            {
                Buttons(enum_iterator),
                button_config
            });

            // Increment enum-iterator
            enum_iterator ++;
        }

        // Function return
        return button_joymap;
    }

    // Joystick Callback-Function
    // -------------------------------
    void JoyCtrl::joystickCB(
        const sensor_msgs::Joy::ConstPtr& msg)
    {   
        // Define temporary variable holder
        geometry_msgs::TwistStamped twist_command;
        control_msgs::JointJog joint_command;

        // Joystick Control-Mode
        // -------------------------------
            // Get Joystick Control Mode
            getJoyControlMode(msg, button_map_, control_mode_);

        // Cartesian Servo-Command
        // -------------------------------
            // Get Servo Twist-Command
            twist_command = getServoCommandTwist(msg, taskspace_map_, control_mode_);

            // Publish Cartesian Servo-Command
            twist_pub_.publish(twist_command);

        // Joint Servo-Command
        // -------------------------------
            // Get Servo Joint-Command
            joint_command = getServoCommandJoint(msg, jointspace_map_, control_mode_);

            // Publish Joint Servo-Command
            joint_pub_.publish(joint_command);
    }

    // Get Servo-Command Twist (Cartesian)
    // -------------------------------
    geometry_msgs::TwistStamped JoyCtrl::getServoCommandTwist(
        const sensor_msgs::Joy::ConstPtr& msg,
        std::map<int, JoyAxisConfig>& taskspace_map,
        const int control_mode)
    {
        // Define temporary variable holder
        geometry_msgs::TwistStamped twiststamped;
        
        // Iterate over Task-Space map
        for(auto it= taskspace_map.begin(); it != taskspace_map.end(); it++)
        {
            // Update value for current Axis of Task-Space map
            updateJoyAxis(msg, taskspace_map[it->first]);  
        }

        // Get current timestamp
        twiststamped.header.stamp = ros::Time::now();

        // Joystick Control Mode
        switch(control_mode) 
        {
            // Mode: Cartesian Translational 
            case MODE_XYZ:
                twiststamped.twist.linear.x = taskspace_map[LINEAR_X].value;
                twiststamped.twist.linear.y = taskspace_map[LINEAR_Y].value;
                twiststamped.twist.linear.z = taskspace_map[LINEAR_Z].value;

                twiststamped.twist.angular.x = 0;
                twiststamped.twist.angular.y = 0;
                twiststamped.twist.angular.z = 0;

                break;

            // Mode:  Cartesian Orientation
            case MODE_RPY:
                twiststamped.twist.linear.x = 0;
                twiststamped.twist.linear.y = 0;
                twiststamped.twist.linear.z = 0;

                twiststamped.twist.angular.x = taskspace_map[ANGULAR_X].value;
                twiststamped.twist.angular.y = taskspace_map[ANGULAR_Y].value;
                twiststamped.twist.angular.z = taskspace_map[ANGULAR_Z].value;

                break;

            // Mode:  Cartesian Translational and Orientation
            case MODE_XYZRPY:
                twiststamped.twist.linear.x = taskspace_map[LINEAR_X].value;
                twiststamped.twist.linear.y = taskspace_map[LINEAR_Y].value;
                twiststamped.twist.linear.z = taskspace_map[LINEAR_Z].value;

                twiststamped.twist.angular.x = taskspace_map[ANGULAR_X].value;
                twiststamped.twist.angular.y = taskspace_map[ANGULAR_Y].value;
                twiststamped.twist.angular.z = taskspace_map[ANGULAR_Z].value;
                
                break;
            
            // Default/Unknown Mode
            default:
                twiststamped.twist.linear.x = 0;
                twiststamped.twist.linear.y = 0;
                twiststamped.twist.linear.z = 0;
                
                twiststamped.twist.angular.x = 0;
                twiststamped.twist.angular.y = 0;
                twiststamped.twist.angular.z = 0;

                break;
        } // End Switch-Case

        // Function return
        return twiststamped;
    }

    // Get Servo-Command Joint
    // -------------------------------
    control_msgs::JointJog JoyCtrl::getServoCommandJoint(
        const sensor_msgs::Joy::ConstPtr& msg,
        std::map<int, JoyAxisConfig>& jointspace_map,
        const int control_mode)
    {
        // Define temporary variable holder
        control_msgs::JointJog jointjog;
        
        // Iterate over Task-Space map
        for(auto it= jointspace_map.begin(); it != jointspace_map.end(); it++)
        {
            // Update value for current Axis of Task-Space map
            updateJoyAxis(msg, jointspace_map[it->first]);  
        }

        // Get current timestamp
        jointjog.header.stamp = ros::Time::now();

        // Joystick Control Mode
        switch(control_mode) 
        {
            // Mode: Joint 1 - Joint 3
            case MODE_JOINT_Q13:
                jointjog.velocities.push_back(jointspace_map[JOINT_1].value);   // Joint 1
                jointjog.velocities.push_back(jointspace_map[JOINT_2].value);   // Joint 2
                jointjog.velocities.push_back(jointspace_map[JOINT_3].value);   // Joint 3

                jointjog.velocities.push_back(0);                               // Joint 4
                jointjog.velocities.push_back(0);                               // Joint 5
                jointjog.velocities.push_back(0);                               // Joint 6

                break;

            // Mode: Joint 4 - Joint 6
            case MODE_JOINT_Q46:
                jointjog.velocities.push_back(0);                               // Joint 1
                jointjog.velocities.push_back(0);                               // Joint 2
                jointjog.velocities.push_back(0);                               // Joint 3

                jointjog.velocities.push_back(jointspace_map[JOINT_4].value);   // Joint 4     
                jointjog.velocities.push_back(jointspace_map[JOINT_5].value);   // Joint 5 
                jointjog.velocities.push_back(jointspace_map[JOINT_6].value);   // Joint 6 

                break;

            // Mode: Joint 1 - Joint 6
            case MODE_JOINT_Q16:
                jointjog.velocities.push_back(jointspace_map[JOINT_1].value);   // Joint 1  
                jointjog.velocities.push_back(jointspace_map[JOINT_2].value);   // Joint 2
                jointjog.velocities.push_back(jointspace_map[JOINT_3].value);   // Joint 3

                jointjog.velocities.push_back(jointspace_map[JOINT_4].value);   // Joint 4 
                jointjog.velocities.push_back(jointspace_map[JOINT_5].value);   // Joint 5 
                jointjog.velocities.push_back(jointspace_map[JOINT_6].value);   // Joint 6 
                
                break;
            
            // Default/Unknown Mode
            default:
                jointjog.velocities.push_back(0);   // Joint 1 
                jointjog.velocities.push_back(0);   // Joint 2
                jointjog.velocities.push_back(0);   // Joint 3
                
                jointjog.velocities.push_back(0);   // Joint 4 
                jointjog.velocities.push_back(0);   // Joint 5 
                jointjog.velocities.push_back(0);   // Joint 6 

                break;
        } // End Switch-Case

        // Verify that Joint-Names equals the size of Joint-Joystick-Commands
        if(joint_names_.size() != jointjog.velocities.size())
        {
            // Report to terminal
            ROS_ERROR_STREAM(msg_prefix_ + "Joint-Names and Joint-Joystick-Commands are not same size!");
        } 

        // Push Names to Joint Servo-Command
        for (int i = 0; i < joint_names_.size(); i++)
        {
            // Joint-Names
            jointjog.joint_names.push_back(joint_names_[i]);
        }

        // Function return
        return jointjog;
    }
        
     // Get Joystick Control Mode
    // -------------------------------
    void JoyCtrl::getJoyControlMode(
        const sensor_msgs::Joy::ConstPtr& msg,
        std::map<int, JoyButtonConfig>& button_map,
        int& control_mode)
    {
        // Defining temporary variable holder
        int control_mode_old = control_mode;    // Update old-value

        // Iterate over Button-Command map
        for(auto it= button_map.begin(); it != button_map.end(); it++)
        {
            // Update value for current Button of Button-Command map
            updateJoyButton(msg, button_map[it->first]);  
        }

        // Increment Control-Mode ID
        if(button_map[BTN_NEXT_MODE].value && !button_map[BTN_NEXT_MODE].value_old)
        {
            // Positive edge trigger on Next-Mode button

            // Increment Control-Mode ID
            control_mode ++;

            // Check if maximum Control-Mode is reached
            if(control_mode > CTRL_MODE_MAP.rbegin()->first)
            {
                // Reset Mode ID to first element of Mode-Map
                control_mode = CTRL_MODE_MAP.begin()->first;
            } 
        } 

        // Decrement Control-Mode ID
        else if(button_map[BTN_PREV_MODE].value && !button_map[BTN_PREV_MODE].value_old)
        {
            // Positive edge trigger on Prev-Mode button

            // Decrement Control-Mode ID
            control_mode --;

            // Check if minimum Control-Mode is reached
            if(control_mode < CTRL_MODE_MAP.begin()->first)
            {
                // Reset Mode ID to last element of Mode-Map
                control_mode = CTRL_MODE_MAP.rbegin()->first;
            } 
        } 

        // Check for updated/new control mode ID
        if (control_mode != control_mode_old)
        {
            // Find Mode-ID name in Control-Mode-Map
            auto it = CTRL_MODE_MAP.find(control_mode);

            // Report to terminal
            ROS_INFO_STREAM(msg_prefix_ + "Control Mode: [" << it->second << "] is active");
            ROS_INFO_STREAM("Control Mode ID: " << control_mode_);
        }
    }

    // Update Joy Axis
    // -------------------------------
    void JoyCtrl::updateJoyAxis(
        const sensor_msgs::Joy::ConstPtr& msg,
        JoyAxisConfig& axis)
    {
        // Get raw axis-value from joystick using index number
        double raw_value = msg->axes[axis.index];

        // Update axis-value with rescaling and deadzone compensation
        axis.value = calcRescaleDeadzone(
                        raw_value,
                        axis.raw_value_min,
                        axis.raw_value_max,
                        axis.deadzone_value,
                        axis.scale_value_min,
                        axis.scale_value_max);
    }

    // Update Joy Button
    // -------------------------------
    void JoyCtrl::updateJoyButton(
        const sensor_msgs::Joy::ConstPtr& msg,
        JoyButtonConfig& button)
    {
        // Update old button-value
        button.value_old = button.value;

        // Update button-value with raw-value from joystick using index number
        button.value = msg->buttons[button.index];
    }

    // Calculate Rescaling
    // -------------------------------
    // (Function overloading)
    double JoyCtrl::calcRescaleDeadzone(
            double raw_value,
            double raw_min,
            double raw_max,
            double raw_deadzone,
            double scale_min,
            double scale_max)
    {
        // Define temporary variable holders
        double tmp_raw_value = 0.0;
        double tmp_raw_min = raw_min + raw_deadzone;
        double tmp_raw_max = raw_max - raw_deadzone;
        double tmp_dz_neg = (-1)*raw_deadzone;
        double tmp_dz_pos = raw_deadzone;
        double value;

        // Deadband Calculation
        // -----------------------------
        // Raw value is whithin deadband range
        if (abs(raw_value) < raw_deadzone)
        {
            tmp_raw_value = 0.0;
        }
            
        // Raw value is below deadband range
        else if (raw_value < tmp_dz_neg)
        {
            tmp_raw_value = raw_value + raw_deadzone;
        }
            
        // Raw value is above deadband range
        else if (raw_value > tmp_dz_pos)
        {
            tmp_raw_value = raw_value - raw_deadzone;
        }

        // Rescaling
        // -----------------------------
        // Scaling input-value to range: [0 , 1]
        tmp_raw_value = (tmp_raw_value - tmp_raw_min) / (tmp_raw_max - tmp_raw_min);

        // Scaling input-value with scaling factor
        value = tmp_raw_value * (scale_max - scale_min) + scale_min;

        // Function return
        return value;
    }
    
} // Namespace: Joystick

    