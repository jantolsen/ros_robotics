// Toolbox Prefix Parameter 
// -------------------------------
// Description:
//      Toolbox for Prefix Paramters
//      Functions for loading and remapping robot specific parameters to correct for robot prefix
//      Applicaple for when having multiple robots in the same scene.
//
// Version:
//  0.1 - Initial Version
//        [10.09.2022]  -   Jan T. Olsen
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
#ifndef PREFIX_PARAM_TOOL_H       
#define PREFIX_PARAM_TOOL_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

// Namespace: Prefix Param Toolbox
// -------------------------------
namespace PrefixParamTool
{

    // Get Joint-Name parameter (loaded together with node)
    // and assigned them according to robot prefix

    // Prefix Joint-Names
    // -------------------------------
    /** \brief Load and remap robot's joint-name parameters
    * (loaded together with node), 
    * and assign joint-names with robot specific prefix
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    */
    void prefixJointNames(ros::NodeHandle nh,
                          std::string robot_prefix);

    // Prefix Joint-Limits
    // -------------------------------
    /** \brief Load and remap robot's joint-limits parameters
    * (loaded together with node), 
    * and assign joint-limits with robot specific prefix
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    */
    void prefixJointLimits(ros::NodeHandle nh,
                           std::string robot_prefix);

    // Prefix Controller-List
    // -------------------------------
    /** \brief Create robot's controller-list parameters
    * with robot specific prefix
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    */
    void prefixControllerList(ros::NodeHandle nh,
                              std::string robot_prefix);

    // Prefix Topic-List
    // -------------------------------
    /** \brief Create robot's topic-list parameters
    * with robot specific prefix
    * (this is special for Yaskawa/Motoman robots)
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    * \param robot_type Robot Type
    */
    void prefixTopicList(ros::NodeHandle nh,
                         std::string robot_prefix,
                         std::string robot_type);

    // Prefix Kinematics-Parameters
    // -------------------------------
    /** \brief Load and remap robot's kinematics parameters
    * (loaded together with node), 
    * and assign kinematics parameters with robot specific prefix
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    */
    void prefixKinematicsParam(ros::NodeHandle nh,
                               std::string robot_prefix);


    // Prefix OMPL-Planning-Parameters
    // -------------------------------
    /** \brief Remap robot's OMPL-Planning parameters
    * (loaded together with planning-pipeline), 
    * and assign OMPL-Planning parameters with robot specific prefix
    * \param nh ROS Nodehandle
    * \param robot_prefix Robot Prefix Name
    */
    void prefixOMPLParam(ros::NodeHandle nh,
                         std::string robot_prefix);


} // End Namespace
#endif // PREFIX_PARAM_TOOL_H 