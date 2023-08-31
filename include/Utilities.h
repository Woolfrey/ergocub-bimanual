/**
 * @file   Utilities.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Header containing useful functions for extracting trajectory information from .ini config files.
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry, Eigen::Vector
#include <iostream>                                                                                 // std::cerr and std::cout
#include <map>                                                                                      // std::map
#include <vector>                                                                                   // std::vector
#include <yarp/os/Bottle.h>                                                                         // yarp::os::Bottle

/**
 * A data structure loading joint trajectory parameters from .ini config files
 */
struct JointTrajectory
{
	std::vector<Eigen::VectorXd> waypoints;                                                     ///< Array of joint positions
	std::vector<double> times;                                                                  ///< Array of times
};

/**
 * Specifies the type of Cartesian motion
 */
enum Type {
	relative,                                                                                   ///< Cartesian motion in the local frame
	absolute                                                                                    ///< Cartesian motion in the global frame
};

/**
 * A data type for loading Cartesian trajectories from .ini config files
 */
struct CartesianMotion
{
	std::vector<Eigen::Isometry3d> waypoints;                                                   ///< An array of poses to move through
	std::vector<double> times;                                                                  ///< Time to reach waypoints
	Type type;                                                                                  ///< Relative or absolute
};

/**
 * Converts a pose represented as a 6-element vector to an Isometry3d object
 */
Eigen::Isometry3d transform_from_vector(const std::vector<double> &input);

/**
 * Extract an SE(3) transfrom from a YARP bottle class and convert to Eigen::Isometry
 */
Eigen::Isometry3d transform_from_bottle(const yarp::os::Bottle *bottle);

/**
 * Extract an array from a YARP bottle class and convert to Eigen::Vector
 */
Eigen::VectorXd vector_from_bottle(const yarp::os::Bottle *bottle);

/**
 * Extract a string from a YARP bottle class
 */
std::vector<std::string> string_from_bottle(const yarp::os::Bottle *bottle);

/**
 * Loads the joint trajectories specified in the .ini config file
 * @param bottle The YARP bottle object where the config data has been loaded
 * @param map The map object that will be used to store the trajectory data for future reference.
 * @return Returns true if there were no problems
 */
bool load_joint_configurations(const yarp::os::Bottle *bottle, std::map<std::string,JointTrajectory> &map);

/**
 * Loads Cartesian trajectories specified in .ini config files
 * @param bottle The YARP object where the data has been loaded to
 * @param nameList The names associated with each of the trajectories
 * @param map The map object where the trajectories will be stored for future reference.
 * @return Returns true if there were no problems.
 */
bool load_cartesian_trajectories(const yarp::os::Bottle *bottle,
                                 const std::vector<std::string> nameList,
                                 std::map<std::string,CartesianMotion> &map);

#endif
