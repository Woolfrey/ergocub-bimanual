    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                       Useful functions                                         //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry, Eigen::Vector
#include <iostream>                                                                                 // std::cerr and std::cout
#include <map>                                                                                      // std::map
#include <vector>                                                                                   // std::vector
#include <yarp/os/Bottle.h>                                                                         // yarp::os::Bottle

// This is to attach the joint positions to the times for a joint trajectory
struct JointTrajectory
{
	std::vector<Eigen::VectorXd> waypoints;
	
	std::vector<double> times;
};

enum Type {relative, absolute};

struct CartesianMotion
{
	std::vector<Eigen::Isometry3d> waypoints;
	
	std::vector<double> times;
	
	Type type;
};

Eigen::Isometry3d transform_from_vector(const std::vector<double> &input);

Eigen::Isometry3d transform_from_bottle(const yarp::os::Bottle *bottle);

Eigen::VectorXd vector_from_bottle(const yarp::os::Bottle *bottle);                                 // Convert a list of floating point numbers to an Eigen::Vector object

std::vector<std::string> string_from_bottle(const yarp::os::Bottle *bottle);                        // Convert a list of strings to a std::vector<std:string>> 

bool load_joint_configurations(const yarp::os::Bottle *bottle, std::map<std::string,JointTrajectory> &map); // Put joint trajectories from the config file in to a std::map

bool load_cartesian_trajectories(const yarp::os::Bottle *bottle,
                                 const std::vector<std::string> nameList,
                                 std::map<std::string,CartesianMotion> &map);

#endif
