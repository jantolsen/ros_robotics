// Planner Descartes 
// -------------------------------
// Description:
//      
//      
//
// Version:
//  0.1 - Initial Version
//        [19.01.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robotics_planner/planner_descartes.h"

// Namespace: Planner
// -------------------------------
namespace Planner
{

    // Class constructor
    // -------------------------------
    PlannerDescartes::PlannerDescartes(
        ros::NodeHandle& nh,
        ros::NodeHandle& pnh)
    :
        nh_(nh),
        pnh_(pnh)
    {
        // Initialize
        init();

    }


    // Class destructor
    // -------------------------------
    PlannerDescartes::~PlannerDescartes()
    {
        // Report to terminal
        ROS_INFO_STREAM(CLASS_PREFIX + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }


    // Class initialization
    // -------------------------------
    void PlannerDescartes::init()
    {
        // Load parameters
        loadParameters();

        // Initialize Publisher(s)
        initPublishers();

        // Initialize Action-Client(s)
        initActionClients();

        // Initialize Robot-Model
        initRobotModel();

        // Compute Trajectory
        std::vector<descartes_core::TrajectoryPtPtr> descartes_trajectory = computeTrajectory();

        // Plan Trajectory
        std::vector<descartes_core::TrajectoryPtPtr> planned_trajectory = planTrajectory(descartes_trajectory);

        // Execute Trajectory
        executeTrajectory(planned_trajectory);
    }


    // Load Parameters
    // -------------------------------
    void PlannerDescartes::loadParameters()
    {
        // Read robot-prefix
        if(!pnh_.getParam("/general/robot_prefix/robot_1", config_.robot_prefix))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(CLASS_PREFIX + "Failed to get parameter (/robot_prefix) from server");

            // Exit
            exit(-1);
        }

        // Read global frame data
        if(!pnh_.getParam("/general/manipulator", config_.group_name) ||
           !pnh_.getParam("/general/global_ref_frame", config_.global_frame) ||
           !pnh_.getParam("/general/robot_frame", config_.robot_frame) ||
           !pnh_.getParam("/general/tool_frame", config_.tool_frame) ||
           !pnh_.getParam("/general/robot_type", config_.robot_type))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(CLASS_PREFIX + "Failed to get frame-data parameters (/general/..) from server");

            // Exit
            exit(-1);
        }

        // Report successful loaded parameters
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ <<
                        ": Sucessfully loaded parameters from server"); 
    }


    // Initialize Publisher(s)
    // -------------------------------
    void PlannerDescartes::initPublishers()
    {
        // Initialize publisher for trajectory visualization marker
        pub_marker_trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(
                                    Planner::VISUALIZE_TRAJECTORY_TOPIC,    // Topic name
                                    QUEUE_LENGTH,                           // Queue size 
                                    true);                                  // Last message will be saved and sent to new subscriber               
    }


    // Initialize Action-Client(s)
    // -------------------------------
    void PlannerDescartes::initActionClients()
    {
        // Execute-Trajectory Action-Client
        // -------------------------------
        // Define the action-client as a shared pointer
        ptr_exec_trajectory_ac_ = std::make_shared<Planner::ExecuteTrajectoryActionClient>(
                                    Planner::EXECUTE_TRAJECTORY_ACTION, 
                                    true);

        // Establish connection with Action-Server
        if(!ptr_exec_trajectory_ac_->waitForServer(ros::Duration(ACTIONSERVER_TIMEOUT)))
        {
            // Report timeout and a failed connection
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Timeout! Failed to establish connection with Action-Client: " 
                            << Planner::EXECUTE_TRAJECTORY_ACTION ); 

            // Exit
            exit(-1);
        }

        // Report Successful established connection
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                        ": Sucessfully established connection with Action-Client: " 
                        << Planner::EXECUTE_TRAJECTORY_ACTION); 
    }


    // Create and Initialize Robot-Model
    // -------------------------------
    void PlannerDescartes::initRobotModel()
    {
        // Define robot model opw-kinematic-parameters
        opw_kinematics::Parameters<double> opw_kinematic_param;

        // Load opw-kinematic-parameters based on robot type
        if(!loadKinematicParameters(config_.robot_type, opw_kinematic_param))
        {
            // Report failed loading of kinematic parameters
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Failed to load OPW Kinematics Parameters for Robot-Type: ("
                            << config_.robot_type << ")"); 
            
            // Exit
            exit(-1);
        }

        // Instantiating robot-model
        // robot_model_.reset(new descartes_moveit::MoveitStateAdapter());
        robot_model_.reset(new descartes_opw_model::OPWMoveitStateAdapter(
                                opw_kinematic_param, 
                                config_.robot_frame, 
                                config_.tool_frame));

        // Enable collision checking
        robot_model_->setCheckCollisions(true);
        
        // Initializing robot-model
        if(!robot_model_->initialize(Planner::ROBOT_DESCRIPTION_PARAM,
                                    config_.group_name,
                                    config_.global_frame,
                                    config_.tool_frame))
        {
            // Report failed initialization of robot-model
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Failed to initialize Descartes Robot-Model"); 
            
            // Exit
            exit(-1);
        }

        // Report Successful initialization of robot-model
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                        ": Descartes Robot-Model initialized"); 

        ROS_INFO(" Robot Type: (%s)", config_.robot_type.c_str());                
        ROS_INFO(" Manipulator Group: (%s)", config_.group_name.c_str());
        ROS_INFO(" Global-Frame: (%s)", config_.global_frame.c_str());
        ROS_INFO(" Tool-Frame: %s)", config_.tool_frame.c_str());
    }

    
    // Compute Descartes Trajectory
    // -------------------------------
    std::vector<descartes_core::TrajectoryPtPtr> PlannerDescartes::computeTrajectory()
    {
        // Initialization
        // -------------------------------
            // Define Eigen-Trajectory
            std::vector<Eigen::Isometry3d> trajectory;

            // Define Translation
            double x;
            double y;
            double z;

            // Define Rotation
            double phi;
            double tht;
            double psi;

        // Initial Segment
        // -------------------------------
            // // Transformation init
            // Eigen::Isometry3d tm0;

            // if(!Toolbox::Kinematics::getCurrentTransform(config_.tool_frame, config_.global_frame, tm0))
            // {
            //     // Report failed initialization of robot-model
            //     ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
            //                     ": Failed to get initial Pose"); 
                
            //     // Exit
            //     exit(-1);
            // }

            // Translation
            x = 1.400;
            y = 0.000;
            z = 1.400;

            // Rotation
            phi = 180.0;
            tht = 0.0;
            psi = 180.0;

            // Transformation
            Eigen::Isometry3d tm0 = Toolbox::Math::transMat(x, y, z,
                                                            phi, tht, psi);

            
        // 1. Segement
        // -------------------------------
            // Translation
            x = 1.400;
            y = -0.500;
            z = 1.400;

            // Rotation
            phi = 180.0;
            tht = 0.0;
            psi = 180.0;

            // Transformation
            Eigen::Isometry3d tm1 = Toolbox::Math::transMat(x, y, z,
                                                            phi, tht, psi);

            
            // Trajectory
            std::vector<Eigen::Isometry3d> traj1 = Toolbox::Trajectory::trajectoryLinear(tm0, tm1, 25);                                                    

        // 2. Segement
        // -------------------------------
            // Translation
            x = 1.400;
            y = -0.500;
            z = 1.000;

            // Rotation
            phi = 180.0;
            tht = 0.0;
            psi = 180.0;

            // Transformation
            Eigen::Isometry3d tm2 = Toolbox::Math::transMat(x, y, z,
                                                            phi, tht, psi);

            
            // Trajectory
            std::vector<Eigen::Isometry3d> traj2 = Toolbox::Trajectory::trajectoryLinear(tm1, tm2, 25);    

        // 3. Segement
        // -------------------------------
            // Circle center position
            x = 1.400;
            y = -0.500;
            z = 1.000;

            Eigen::Vector3d center(x, y, z);
            double radius = 0.250;
            double angle = Toolbox::Common::degToRad(155.0);

            // Trajectory
            std::vector<Eigen::Isometry3d> traj3 = Toolbox::Trajectory::trajectoryCircular(center, radius, angle, 50);  

        // 4. Segement
        // -------------------------------
            
            // Trajectory
            std::vector<Eigen::Isometry3d> traj4 = Toolbox::Trajectory::trajectoryLinear(tm2, tm1, 25);    

        // Combine Segments
        // -------------------------------
            trajectory.insert(trajectory.end(), traj1.begin(), traj1.end());
            trajectory.insert(trajectory.end(), traj2.begin(), traj2.end());
            trajectory.insert(trajectory.end(), traj3.begin(), traj3.end());
            trajectory.insert(trajectory.end(), traj4.begin(), traj4.end());

        // Visualize Trajectory
        // -------------------------------
            // // Compute Visualization Marker-Array for Trajectory
            // visualization_msgs::MarkerArray trajectory_visualize_msg = Toolbox::Visual::visualPoseTrajectory(trajectory, 0.01, 0.10);

            // // DEBUG
            // // Publish Visualization Trajectory
            // pub_marker_trajectory_.publish(trajectory_visualize_msg);

        // Descartes Trajectory
        // -------------------------------
            // Define Descartes-Trajectory
            std::vector<descartes_core::TrajectoryPtPtr> descartes_trajectory;
            descartes_trajectory.clear();
            descartes_trajectory.reserve(trajectory.size());

            // Configure to use End-Effector TCP as target
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);

            // Query listener for specific transformation
            auto tcp_frame = tf_buffer.lookupTransform("robot_tcp",          // Reference Frame to which data should be transformed relative to
                                                config_.tool_frame,     // Target Frame which data originates
                                                ros::Time(0),           // Time at which value of transformed is desired (0 will get latest data)
                                                ros::Duration(0.5));    // Duration before timeout 
                                                    
            // Convert to Eigen
            Eigen::Isometry3d tcp = tf2::transformToEigen(tcp_frame);

            descartes_core::Frame wobj_base = descartes_core::Frame::Identity();
            descartes_trajectory::TolerancedFrame wobj_pt = descartes_core::Frame::Identity(); 

            descartes_core::Frame tool_base(tcp);
            descartes_trajectory::TolerancedFrame tool_pt = descartes_core::Frame::Identity();

            // // Iterate over linaer-trajectory
            // for(unsigned int i = 0; i < trajectory.size(); i ++)
            // {
            //     // Get Pose for each element in trajectory
            //     const Eigen::Isometry3d& pose = trajectory[i];

            //     // Assign Work-Object Point
            //     descartes_trajectory::TolerancedFrame wobj_pt(pose);

            //     // Time Constant
            //     double time_constant = 0.1;

            //     // Special point
            //     if ((i <= 1) || (i >= 49 && i<=51) || (i >= 100 && i<=101))
            //     // if ((i <= 1) || (i >= 49 && i<=51) || (i >= 75 && i<=76))
            //     {
            //         time_constant = 0.0;
            //     }
            //     else if(i >= 50 && i<=101)
            //     // else if(i >= 50 && i<=76)
            //     {
            //         time_constant = 0.5;
            //     }

            //     // Convert pose to descartes-point
            //     descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
            //         new descartes_trajectory::CartTrajectoryPt(
            //                 wobj_base, 
            //                 wobj_pt, 
            //                 tool_base, 
            //                 tool_pt,
            //                 0,
            //                 0,
            //                 descartes_core::TimingConstraint(time_constant)
            //             )
            //         );

            //     // Append descartes-point to descartes-trajectory
            //     descartes_trajectory.push_back(pt);
            // }


            // TEST
            // -----------------------------------------------------------------------------------------
                // // // descartes_core::TrajectoryPtPtr pt_test;

                // // // // Assign Work-Object Point
                // // // // descartes_trajectory::TolerancedFrame wobj_pt;

                // // // // Time Constant
                // // // double time_constant = 1.0;
                
                // // // // init:
                // // // wobj_pt = descartes_core::Frame::Identity();

                // // // // Convert pose to descartes-point
                // // // descartes_core::TrajectoryPtPtr pt_init = descartes_core::TrajectoryPtPtr(
                // // //     new descartes_trajectory::CartTrajectoryPt(
                // // //             wobj_base, 
                // // //             wobj_pt, 
                // // //             tool_base, 
                // // //             tool_pt,
                // // //             0,
                // // //             0,
                // // //             descartes_core::TimingConstraint(time_constant)
                // // //         )
                // // //     );

                // // // // Append descartes-point to descartes-trajectory
                // // // descartes_trajectory.push_back(pt_init);

                // // // // 0:
                // // // wobj_pt = tm0;

                // // // // Convert pose to descartes-point
                // // // descartes_core::TrajectoryPtPtr pt_0 = descartes_core::TrajectoryPtPtr(
                // // //     new descartes_trajectory::CartTrajectoryPt(
                // // //             wobj_base, 
                // // //             wobj_pt, 
                // // //             tool_base, 
                // // //             tool_pt,
                // // //             0,
                // // //             0,
                // // //             descartes_core::TimingConstraint(time_constant)
                // // //         )
                // // //     );

                // // // // Append descartes-point to descartes-trajectory
                // // // descartes_trajectory.push_back(pt_0);

                // // // // 1:
                // // // wobj_pt = tm1;

                // // // // Convert pose to descartes-point
                // // // descartes_core::TrajectoryPtPtr pt_1 = descartes_core::TrajectoryPtPtr(
                // // //     new descartes_trajectory::CartTrajectoryPt(
                // // //             wobj_base, 
                // // //             wobj_pt, 
                // // //             tool_base, 
                // // //             tool_pt,
                // // //             0,
                // // //             0,
                // // //             descartes_core::TimingConstraint(time_constant)
                // // //         )
                // // //     );

                // // // // Append descartes-point to descartes-trajectory
                // // // descartes_trajectory.push_back(pt_1);

                // // // // 2:
                // // // wobj_pt = tm2;

                // // // // Convert pose to descartes-point
                // // // descartes_core::TrajectoryPtPtr pt_2 = descartes_core::TrajectoryPtPtr(
                // // //     new descartes_trajectory::CartTrajectoryPt(
                // // //             wobj_base, 
                // // //             wobj_pt, 
                // // //             tool_base, 
                // // //             tool_pt,
                // // //             0,
                // // //             0,
                // // //             descartes_core::TimingConstraint(time_constant)
                // // //         )
                // // //     );

                // // // // Append descartes-point to descartes-trajectory
                // // // descartes_trajectory.push_back(pt_2);

            // ----------------------------------------------------------------------------------------

                // 0. Segement
                // -------------------------------
                // Translation
                x = 1.400;
                y = 0.000;
                z = 1.400;

                // Rotation
                phi = 180.0;
                tht = 0.0;
                psi = 180.0;

                // Transformation
                tm0 = Toolbox::Math::transMat(x, y, z,
                                            phi, tht, psi);

                
                // 1. Segement
                // -------------------------------
                // Translation
                x = 1.400;
                y = -0.500;
                z = 1.400;

                // Rotation
                phi = 180.0;
                tht = 0.0;
                psi = 180.0;

                // Transformation
                tm1 = Toolbox::Math::transMat(x, y, z,
                                            phi, tht, psi);

                
                // Trajectory
                std::vector<Eigen::Isometry3d> traj_test = Toolbox::Trajectory::trajectoryLinear(tm0, tm1, 2);     

                // Visualize Trajectory
                // -------------------------------
                    // Compute Visualization Marker-Array for Trajectory
                    visualization_msgs::MarkerArray traj_test_visualize_msg = Toolbox::Visual::visualPoseTrajectory(traj_test, 0.01, 0.10);

                    // Publish Visualization Trajectory
                    pub_marker_trajectory_.publish(traj_test_visualize_msg);


                for(unsigned int i = 0; i < traj_test.size(); i ++)
                {
                    // Get Pose for each element in trajectory
                    const Eigen::Isometry3d& pose = traj_test[i];

                    // Assign Work-Object Point
                    descartes_trajectory::TolerancedFrame wobj_pt(pose);

                    // Convert pose to descartes-point
                    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                        new descartes_trajectory::CartTrajectoryPt(
                                wobj_base, 
                                wobj_pt, 
                                tool_base, 
                                tool_pt,
                                0,
                                0,
                                descartes_core::TimingConstraint(2.0)
                            )
                        );

                    // Append descartes-point to descartes-trajectory
                    descartes_trajectory.push_back(pt);
                }

            // ----------------------------------------------------------------------------------------
            // TEST

            
            // Start Descartes Trajectory at current robot position
            std::vector<double> current_joint_state;
            
            // Get Current Joint-States
            if(!Toolbox::Kinematics::getCurrentJointState("joint_states", current_joint_state))
            {
                // Report failed to get current joint-state
                ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                                ": Failed to get current joint-state"); 
                
                // Exit
                exit(-1);
            }

            // Convert Joint-State to descartes-point
            descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                new descartes_trajectory::JointTrajectoryPt(
                    current_joint_state, 
                    descartes_core::TimingConstraint(0.0)
                    )
                );

            // Append Current Position to front of trajectory     
            // descartes_trajectory.front() = pt;
            descartes_trajectory.insert(descartes_trajectory.begin(), pt);

        // Report Successful Trajectory computation
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                        ": Trajectory calculated"); 

        // DEBUG
        ROS_ERROR("TRAJECTORY SIZE: (%zd)", descartes_trajectory.size());

        // Function return
        return descartes_trajectory;
    }


    // Plan Descartes Trajectory
    // -------------------------------
    std::vector<descartes_core::TrajectoryPtPtr> PlannerDescartes::planTrajectory(
        const std::vector<descartes_core::TrajectoryPtPtr>& trajectory)
    {
        // Instantiating Descartes Path Planner
        descartes_planner::DensePlanner descartesPlanner;

        // Initialize Descartes Path Planner
        if(!descartesPlanner.initialize(robot_model_))
        {
            // Report failed initialization of Descartes Path Planner
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Failed to initialize Descartes Planner"); 
            
            // Exit
            exit(-1);
        }

        // Report Successful initialization of Descartes Path Planner
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                        ": Descartes Planner initialized"); 

        // Plan descartes trajectory
        if (!descartesPlanner.planPath(trajectory))
        {
            // Report failed path-planning of Descartes Path
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Failed to solve for given path"); 

            // Error code report
            std::string error_msg;
            int error_code = descartesPlanner.getErrorCode();
            descartesPlanner.getErrorMessage(error_code, error_msg);
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Descartes planning failed with error msg: " << error_msg << "(" << error_code << ")");

            // Exit
            exit(-1);
        }
        
        // Report Successful path-planning of Descartes Path
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                ": Descartes Trajecory sucessfully planned"); 

        // Acquire Descartes Planner result
        std::vector<descartes_core::TrajectoryPtPtr> result_trajectory;

        if (!descartesPlanner.getPath(result_trajectory))
        {
            // Report failed attempt to retrieve Descartes Path
            ROS_ERROR_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                            ": Failed to retrieve planned path"); 
            
            // Exit
            exit(-1);
        }

        // Report Successful solving for Descartes Path
        ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ << 
                        ": Descartes Trajecory sucessfully retrieved"); 

        // Function return
        return result_trajectory;
    }


    // Execute Trajectory
    // -------------------------------
    void PlannerDescartes::executeTrajectory(
        const std::vector<descartes_core::TrajectoryPtPtr>& trajectory)
    {
        // Initialize
        std::vector<std::string> joint_names;
        nh_.getParam("controller_joint_names", joint_names);

        // Create a JointTrajectory
        trajectory_msgs::JointTrajectory joint_solution;
        joint_solution.joint_names = joint_names;

        // Define a default velocity. Descartes points without specified timing will use this value to limit the
        // fastest moving joint. This usually effects the first point in your path the most.
        const static double default_joint_vel = 0.5; // rad/s
        if (!descartes_utilities::toRosJointPoints(*robot_model_, trajectory, default_joint_vel, joint_solution.points))
        {
            ROS_ERROR("Unable to convert Descartes trajectory to joint points");
            
            // Exit
            exit(-1);
        }

        // // Create a JointTrajectory
        // trajectory_msgs::JointTrajectory joint_solution_new;
        // joint_solution_new.joint_names = joint_names;
        // joint_solution_new.points = joint_solution.points;

        // for (int i = 51; i < joint_solution_new.points.size() - 26; i++)
        // {
        //     joint_solution_new.points.at(i).positions.at(5) = Toolbox::Common::degToRad(i*25);
        // }

        // Define Move-It Trajectory
        moveit_msgs::RobotTrajectory moveit_trajectory;
        moveit_trajectory.joint_trajectory = joint_solution;
        // moveit_trajectory.joint_trajectory = joint_solution_new;

        moveit_msgs::ExecuteTrajectoryGoal goal;
        goal.trajectory = moveit_trajectory;

        ROS_INFO_STREAM("Robot path sent for execution");
        if(ptr_exec_trajectory_ac_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO_STREAM("Robot path execution completed");
        }
        else
        {
            ROS_ERROR_STREAM("Failed to run robot path with error "<<*ptr_exec_trajectory_ac_->getResult());
            exit(-1);
        }

        ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");
    }

} // Namespace: Planner