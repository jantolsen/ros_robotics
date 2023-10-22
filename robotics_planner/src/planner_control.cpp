// Planner Control 
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
    #include "robotics_planner/planner_control.h"

// Namespace: Planner
// -------------------------------
namespace Planner
{

    // Class constructor
    // -------------------------------
    PlannerControl::PlannerControl(
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
    PlannerControl::~PlannerControl()
    {
        // Report to terminal
        ROS_INFO_STREAM(class_prefix_ + "Destructor called. ROS Shutdown");

        // ROS Shutdown
        ros::shutdown();
    }    

    // Class initialization
    // -------------------------------
    void PlannerControl::init()
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
        computeDescartesTrajectory();

    }

    // Load Parameters
    // -------------------------------
    void PlannerControl::loadParameters()
    {
        // Read robot-prefix
        if(!pnh_.getParam("/general/robot_prefix/robot_1", config_.robot_prefix))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(class_prefix_ + "Failed to get parameter (/robot_prefix) from server");

            // Exit
            exit(-1);
        }

        // Read global frame data
        if(!pnh_.getParam("/general/manipulator", config_.group_name) ||
           !pnh_.getParam("/general/global_ref_frame", config_.global_frame) ||
           !pnh_.getParam("/general/robot_frame", config_.robot_frame) ||
           !pnh_.getParam("/general/tool_frame", config_.tool_frame))
        {
            // Report if Get Parameter failed
            ROS_ERROR_STREAM(class_prefix_ + "Failed to get frame-data parameters (/general/..) from server");

            // Exit
            exit(-1);
        }

        // Report successful loaded parameters
        ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << ": Sucessfully loaded parameters from server"); 
    }

    // Initialize Publisher(s)
    // -------------------------------
    void PlannerControl::initPublishers()
    {
        // Initialize publisher for trajectory visualization marker
        pub_marker_trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(
                                    Planner::VISUALIZE_TRAJECTORY_TOPIC,     // Topic name
                                    QUEUE_LENGTH,                   // Queue size 
                                    true);                          // Last message will be saved and sent to new subscriber               
    }

    // Initialize Action-Client(s)
    // -------------------------------
    void PlannerControl::initActionClients()
    {
        // Execute-Trajectory Action-Client
        // -------------------------------
        // Define the action-client as a shared pointer
        ptr_exec_trajectory_ac_ = std::make_shared<Planner::ExecuteTrajectoryActionClient>(Planner::EXECUTE_TRAJECTORY_ACTION, true);

        // Establish connection with Action-Server
        if(!ptr_exec_trajectory_ac_->waitForServer(ros::Duration(ACTIONSERVER_TIMEOUT)))
        {
            // Report timeout and a failed connection
            ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                            ": Timeout! Failed to establish connection with Action-Client: " << Planner::EXECUTE_TRAJECTORY_ACTION ); 

            // Exit
            exit(-1);
        }

        // Report Successful established connection
        ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Sucessfully established connection with Action-Client: " << Planner::EXECUTE_TRAJECTORY_ACTION); 
    }

    // Create and Initialize Robot-Model
    // -------------------------------
    void PlannerControl::initRobotModel()
    {
        const auto abb_irb6660 = opw_kinematics::makeABB_IRB6660<double>();

        // Instantiating robot-model
        // ptr_robot_model_.reset(new descartes_moveit::MoveitStateAdapter());
        // ptr_robot_model_.reset(new descartes_moveit::IkFastMoveitStateAdapter());
        ptr_robot_model_.reset(new descartes_opw_model::OPWMoveitStateAdapter(abb_irb6660, config_.robot_frame, config_.tool_frame));

        ptr_robot_model_->setCheckCollisions(true);
        
        // Initializing robot-model
        if(!ptr_robot_model_->initialize(Planner::ROBOT_DESCRIPTION_PARAM,
                                         config_.group_name,
                                         config_.global_frame,
                                         config_.tool_frame))
        {
            // Report failed initialization of robot-model-ptr
            ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                            ": Failed to initialize Descartes Robot-Model"); 
            
            // Exit
            exit(-1);
        }

        

        // Report Successful initialization of robot-model-ptr
        ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Descartes Robot-Model initialized"); 

        ROS_WARN_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Group-Name: " << config_.group_name);

        ROS_WARN_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Global-Frame: " << config_.global_frame);


        ROS_WARN_STREAM(class_prefix_ << __FUNCTION__ << 
                        ": Tool-Frame: " << config_.tool_frame);
    }

    // Compute Descartes Trajectory
    // -------------------------------
    void PlannerControl::computeDescartesTrajectory()
    {
        // Transformation Start
        // -------------------------------
            // Translation
            // double x0 = 1.393; // + 0.260;
            double x0 = 1.500;
            double y0 = 0.000;
            // double z0 = 1.794;
            double z0 = 1.500;
            // double x0 = 0.000;
            // double y0 = 0.000;
            // double z0 = 0.000;
            Eigen::Vector3d pos0(x0, y0, z0);

            // Rotation
            double phi0 = 0;
            double theta0 = 90;
            double psi0 = 0;
            Eigen::Vector3d rot0(Toolbox::Common::degToRad(phi0), 
                                Toolbox::Common::degToRad(theta0), 
                                Toolbox::Common::degToRad(psi0));

            // Transformation
            Eigen::Isometry3d tm0 = Toolbox::Math::transMat(pos0, rot0);
            
        // Transformation End
        // -------------------------------
            // Translation
            double x1 = 1.650; // + 0.260;
            double y1 = 0.000;
            double z1 = 0.650;
            // double z1 = 0.350;
            Eigen::Vector3d pos1(x1, y1, z1);

            // Rotation
            double phi1 = 0;
            double theta1 = 90;
            // double psi1 = 90;
            double psi1 = 270;
            Eigen::Vector3d rot1(Toolbox::Common::degToRad(phi1), 
                                Toolbox::Common::degToRad(theta1), 
                                Toolbox::Common::degToRad(psi1));

            // Transformation
            Eigen::Isometry3d tm1 = Toolbox::Math::transMat(pos1, rot1);

        // Trajectory
        // -------------------------------
            int steps = 200;
            std::vector<Eigen::Isometry3d> trajectory_linear = Toolbox::Trajectory::trajectoryLinear(tm0, tm1, steps);

        
        // Visualize Trajectory
        // -------------------------------
            // Compute Visualization Marker-Array for Trajectory
            visualization_msgs::MarkerArray trajectory_visualize_msg = Toolbox::Visual::visualPoseTrajectory(trajectory_linear, 0.01, 0.10);

            // Publish Visualization Trajectory
            pub_marker_trajectory_.publish(trajectory_visualize_msg);

        // Descartes Trajectory
        // -------------------------------
            std::vector<descartes_core::TrajectoryPtPtr> descartesTrajectory;
            descartesTrajectory.clear();
            descartesTrajectory.reserve(trajectory_linear.size());

            // descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

            // // Translation
            // double x_wb = 1.393;
            // double y_wb = 0.000;
            // double z_wb = 1.794;
            // Eigen::Vector3d pos_wb(x_wb, y_wb, z_wb);

            // // Rotation
            // double phi_wb = 0;
            // double theta_wb = 90;
            // double psi_wb = 0;
            // Eigen::Vector3d rot_wb(Toolbox::Common::degToRad(phi_wb), 
            //                         Toolbox::Common::degToRad(theta_wb), 
            //                         Toolbox::Common::degToRad(psi_wb));

            // // Transformation
            // Eigen::Isometry3d wobj = Toolbox::Math::transMat(pos_wb, rot_wb);

            // // descartes_core::Frame wobj_base(wobj); 
            // descartes_core::Frame wobj_base = descartes_core::Frame::Identity();
            // descartes_core::Frame tool_base = descartes_core::Frame::Identity();
            // descartes_trajectory::TolerancedFrame wobj_pt = descartes_core::Frame::Identity();;

            
            tf::TransformListener listener;
            tf::StampedTransform tcp_frame;

            // listener.waitForTransform("robot_tcp", "robot_eef_tcp", ros::Time(0), ros::Duration(5.0));
            // listener.lookupTransform("robot_tcp", "robot_eef_tcp", ros::Time(0), tcp_frame);
            listener.waitForTransform("robot_tool0", "robot_eef_tcp", ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("robot_tool0", "robot_eef_tcp", ros::Time(0), tcp_frame);

            // Descartes uses eigen, so let's convert the data type
            Eigen::Isometry3d tcp;
            tf::transformTFToEigen(tcp_frame, tcp);


            descartes_core::Frame wobj_base = descartes_core::Frame::Identity();
            // descartes_core::Frame wobj_base(tcp);
            descartes_trajectory::TolerancedFrame wobj_pt = descartes_core::Frame::Identity(); 
            // descartes_trajectory::TolerancedFrame wobj_pt(tcp); 

            // descartes_core::Frame tool_base = descartes_core::Frame::Identity();
            descartes_core::Frame tool_base(tcp);
            descartes_trajectory::TolerancedFrame tool_pt = descartes_core::Frame::Identity();


            // Iterate over linaer-trajectory
            for(unsigned int i = 0; i < trajectory_linear.size(); i ++)
            {
                // Get Pose for each element in trajectory
                const Eigen::Isometry3d& pose = trajectory_linear[i];
                descartes_core::Frame pose_frame(pose);

                descartes_trajectory::TolerancedFrame wobj_pt(pose);


                // descartes_trajectory::TolerancedFrame tool_pt(pose);
                // tool_pt.orientation_tolerance.z_lower = -Toolbox::Common::degToRad(0.1); // Search -PI to PI (so 360 degrees)
                // tool_pt.orientation_tolerance.z_upper = Toolbox::Common::degToRad(0.1);

                



                // tool_pt.orientation_tolerance = descartes_trajectory::TolerancedFrame::orientation_tolerance.zer;


                // Convert pose to descartes-point
                //  boost::shared_ptr<descartes_trajectory::CartTrajectoryPt> pt(new descartes_trajectory::CartTrajectoryPt(
                //                                                 wobj_base, wobj_pt, tool_base, tool_pt,
                //                                                 0,
                //                                                 M_PI/(180 / 10), 
                //                                                 descartes_core::TimingConstraint(0.25)));

                // descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                //                                         new descartes_trajectory::CartTrajectoryPt(
                //                                                 descartes_trajectory::TolerancedFrame(pose), 
                //                                                 descartes_core::TimingConstraint(0.25)));

                // descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                //                                         new descartes_trajectory::CartTrajectoryPt(
                //                                                 tool_pt,
                //                                                 0.001,
                //                                                 Toolbox::Common::degToRad(0.01), 
                //                                                 descartes_core::TimingConstraint(0.25)));

                // if (i >= 1)
                // {
                //     descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                //                                         new descartes_trajectory::CartTrajectoryPt(
                //                                                 pose_frame, 
                //                                                 descartes_core::TimingConstraint(0.0)));
                
                //     // Append descartes-point to descartes-trajectory
                //     descartesTrajectory.push_back(pt);
                // }
                

                // else
                // {
                //     descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                //                                         new descartes_trajectory::AxialSymmetricPt(
                //                                                 pose,
                //                                                 0.5,
                //                                                 descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS));
                //     // Append descartes-point to descartes-trajectory
                //     descartesTrajectory.push_back(pt);
                // }
                double time_constant = 0.1;
                if (i<=1)
                {
                    time_constant = 0.0;
                }
                descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                    new descartes_trajectory::CartTrajectoryPt(
                            wobj_base, 
                            wobj_pt, 
                            tool_base, 
                            tool_pt,
                            0,
                            0,
                            descartes_core::TimingConstraint(time_constant)
                        )
                    );

                // descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
                //     new descartes_trajectory::CartTrajectoryPt(
                //         pose_frame, 
                //         descartes_core::TimingConstraint(0.0)));

                // Append descartes-point to descartes-trajectory
                descartesTrajectory.push_back(pt);

                
            }

            // Step 3: Tell Descartes to start at the "current" robot position
            std::vector<double> start_joints;
            bool res = Toolbox::Kinematics::getCurrentJointState("joint_states", start_joints);
            descartes_core::TrajectoryPtPtr pt (new descartes_trajectory::JointTrajectoryPt(start_joints, descartes_core::TimingConstraint(0.0)));
            descartesTrajectory.front() = pt;


        // std::vector<double> start_pose, end_pose;
        // std::vector<double> joint_seed = {0, 0, 0, 0, 0, 0};
        // if(descartesTrajectory.front()->getClosestJointPose(joint_seed,*ptr_robot_model_,start_pose) &&
        //     descartesTrajectory.back()->getClosestJointPose(joint_seed,*ptr_robot_model_,end_pose))
        // {
        //     ROS_INFO_STREAM("Setting trajectory start and end to JointTrajectoryPts");

        //     // Creating Start JointTrajectoryPt from start joint pose
        //     descartes_core::TrajectoryPtPtr start_joint_point = descartes_core::TrajectoryPtPtr(
        //         new descartes_trajectory::JointTrajectoryPt(start_pose));

        //     // Creating End JointTrajectoryPt from end joint pose
        //     descartes_core::TrajectoryPtPtr end_joint_point = descartes_core::TrajectoryPtPtr(
        //         new descartes_trajectory::JointTrajectoryPt(end_pose));

        //     // Modifying start and end of the trajectory.
        //     descartesTrajectory[0] = start_joint_point;
        //     descartesTrajectory[descartesTrajectory.size() - 1 ] = end_joint_point;
        // }
        // else
        // {
        //     ROS_ERROR_STREAM("Failed to find closest joint pose to seed pose at the start or end of trajectory");
        //     exit(-1);
        // }

        // Plan Descartes Trajectory
        // -------------------------------
            // Instantiating Descartes Path Planner
            descartes_planner::DensePlanner descartesPlanner;

            // Initialize Descartes Path Planner
            if(!descartesPlanner.initialize(ptr_robot_model_))
            {
                // Report failed initialization of Descartes Path Planner
                ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                                ": Failed to initialize Descartes Planner"); 
                
                // Exit
                exit(-1);
            }

            // Report Successful initialization of Descartes Path Planner
            ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                            ": Descartes Planner initialized"); 

            // for (int tries = 0; tries < 5; tries ++)
            // {   
            //     // Plan descartes trajectory
            //     if (!descartesPlanner.planPath(descartesTrajectory))
            //     {
            //         // Report failed path-planning of Descartes Path
            //         ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
            //                         ": Failed to solve for given path, trying again"); 
                    
            //         // retry
            //         continue;
            //     }
            //     else
            //     {
            //         // Report Successful path-planning of Descartes Path
            //         ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
            //                 ": Descartes Trajecory sucessfully planned"); 

            //         break;
            //     }

            //     // Report failed path-planning of Descartes Path
            //     ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
            //                     ": Failed to solve for given path"); 

            //     // Exit
            //     exit(-1);
            // }
            
            // Plan descartes trajectory
            if (!descartesPlanner.planPath(descartesTrajectory))
            {
                // Report failed path-planning of Descartes Path
                ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                                ": Failed to solve for given path"); 

                std::string error_msg;
                int error_code = descartesPlanner.getErrorCode();
                descartesPlanner.getErrorMessage(error_code, error_msg);
                ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                                ": Descartes planning failed with error msg: " << error_msg << "(" << error_code << ")");

                // Exit
                exit(-1);
            }
            
            // Report Successful path-planning of Descartes Path
            ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                    ": Descartes Trajecory sucessfully planned"); 
            
            

            // Acquire Descartes Planner result
            std::vector<descartes_core::TrajectoryPtPtr> resultTrajectory;

            if (!descartesPlanner.getPath(resultTrajectory))
            {
                // Report failed attempt to retrieve Descartes Path
                ROS_ERROR_STREAM(class_prefix_ << __FUNCTION__ << 
                                ": Failed to retrieve planned path"); 
                
                // Exit
                exit(-1);
            }

            // Report Successful solving for Descartes Path
            ROS_INFO_STREAM(class_prefix_ << __FUNCTION__ << 
                            ": Descartes Trajecory sucessfully retrieved"); 

            executeTrajectory(resultTrajectory);

    }

    void PlannerControl::executeTrajectory(
        const std::vector<descartes_core::TrajectoryPtPtr>& trajectory)
    {
        std::vector<std::string> names;
        nh_.getParam("controller_joint_names", names);

        // Create a JointTrajectory
        trajectory_msgs::JointTrajectory joint_solution;
        joint_solution.joint_names = names;

        // Define a default velocity. Descartes points without specified timing will use this value to limit the
        // fastest moving joint. This usually effects the first point in your path the most.
        const static double default_joint_vel = 0.5; // rad/s
        if (!descartes_utilities::toRosJointPoints(*ptr_robot_model_, trajectory, default_joint_vel, joint_solution.points))
        {
            ROS_ERROR("Unable to convert Descartes trajectory to joint points");
            
            // Exit
            exit(-1);
        }

        moveit_msgs::RobotTrajectory moveit_trajectory;
        moveit_trajectory.joint_trajectory = joint_solution;

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