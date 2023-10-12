/**
 * @file   ControlServer.cpp
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Class enabling bimanual control of the ergoCub through thrift communication.
 */

#include <ControlInterface.h>                                                                       // thrift-generated class
#include <iostream>                                                                                 // std::cerr, std::cout
#include <map>                                                                                      // std::map
#include <BimanualControl.h>                                                                        // For control of ergoCub, iCub robots
#include <Utilities.h>                                                                              // JointTrajectory object structure
#include <yarp/os/Property.h>                                                                       // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

/**
 * A class for parsing control actions for the ergoCub upper body.
 */
class ControlServer : public ControlInterface
{
	public:
	
		/**
		 * Constructor.
		 * @param *_robot A pointer to a previously created BimanualControl object.
		 * @param *_jointActionMap A pointer to a std::map specifying named joint actions.
		 * @param *_leftHandMap A pointer to a std::map specifying Cartesian actions for the left hand.
		 * @param *_rightHandMap A pointer to a std::map specifying Cartesian actions for the right hand.
		 * @param *_graspActionMap A pointer to a std::map specifying actions for manipulating an object with 2 hands.
		 */
		ControlServer(BimanualControl *_robot,
		              std::map<std::string,JointTrajectory> *_jointActionMap,
		              std::map<std::string,CartesianMotion> *_leftHandMap,
		              std::map<std::string,CartesianMotion> *_rightHandMap,
		              std::map<std::string,CartesianMotion> *_graspActionMap)
		             :
		             robot(_robot),
		             jointActionMap(_jointActionMap),
		             leftHandMap(_leftHandMap),
		             rightHandMap(_rightHandMap),
		             graspActionMap(_graspActionMap) {}        
		
		
		////////// NOTE: These are directly copied from CommandInterface.h //////////

		/**
		 * Activate grasping mode so the hands move together when holding an object.
		 * @return Returns true if there were no problems.
		 */
		bool activate_grasp()
		{
			return this->robot->activate_grasp();
		}

		/**
		 * Queries if the robot has finished executing an action.
		 * @return Returns true when finished.
		 */
		bool is_finished() { return this->robot->is_finished(); }
		
		/**
		 * Move the hands (independently) by prescribed action.
		 * @param actionName The name of the action specified in the std::map object for Cartesian actions.
		 * @return Retruns true if there were no issues.
		 */
		bool perform_cartesian_action(const std::string& actionName)
		{
			if(this->robot->is_grasping())
			{
				std::cout << "[ERROR] [CONTROL SERVER] perform_cartesian_action(): "
				          << "The robot is currently grasping something. "
				          << "You need to call 'deactivate_grasp()' to release it.\n";
				
				return false;
			}
			
			// Variables used in this scope
			std::vector<Eigen::Isometry3d> leftWaypoints, rightWaypoints;               // Cartesian waypoints for the hands
			std::vector<double> times;                                                  // Time to reach each waypoint
			Type type;                                                                  // Enumeration; relative or absolute
			
			// Find the left-hand motion in the list
			auto temp = this->leftHandMap->find(actionName);
			
			if(temp == this->leftHandMap->end())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_hands_by_action(): "
				          << "Could not find the action named '" << actionName
				          << "' for the left hand.\n";
				
				return false;
			}
			else
			{
				leftWaypoints = temp->second.waypoints;
				times         = temp->second.times;
				type          = temp->second.type;
			}
			
			// Find the right-hand motion in the list
			temp = this->rightHandMap->find(actionName);
			
			if(temp == this->rightHandMap->end())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_hands_by_action(): "
				          << "Could not find the action named '" << actionName
				          << "' for the right hand.\n";
				
				return false;
			}
			else	rightWaypoints = temp->second.waypoints;
			
			if(type == absolute)
			{
				return this->robot->move_to_poses(leftWaypoints, rightWaypoints, times);
			}
			else // type == relative
			{
				// We need to convert the relative motions to absolute motions before passing on
				std::vector<Eigen::Isometry3d> newLeftPoints, newRightPoints;
				
				newLeftPoints.push_back(this->robot->hand_pose("left")*leftWaypoints[0]);
				
				newRightPoints.push_back(this->robot->hand_pose("right")*rightWaypoints[0]);
				
				for(int i = 1; i < leftWaypoints.size(); i++)
				{
					newLeftPoints.push_back(newLeftPoints[i-1]*leftWaypoints[i]);
					newRightPoints.push_back(newRightPoints[i-1]*rightWaypoints[i]);
				}
				
				return this->robot->move_to_poses(newLeftPoints,newRightPoints,times);
			}  
		}

		/**
		 * Move the hands as together as though they were grasping an object.
		 * @param actionName The name of the grasping action loaded from the config file.
		 * @return Returns true if there were no problems.
		 */
		bool perform_grasp_action(const std::string& actionName)
		{
			// Variables in this scope
			std::vector<Eigen::Isometry3d> objectWaypoints;
			std::vector<double> waypointTimes;
			Type type;                                                                  // Relative or absolute motion
			
			if(not this->robot->is_grasping())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] perform_grasp_action(): "
				          << "Not currently holding anything!\n";
				
				return false;
			}
			
			auto temp = graspActionMap->find(actionName);                               // Temporary placeholder for the iterator
			
			if(temp == graspActionMap->end())
			{
				std::cerr << "[ERROR] [CONTROL SERVER} perform_grasp_action(): "
				          << "Could not find the action named '"
				          << actionName << "' in the list.\n";
				          
				return false;
			}

			objectWaypoints = temp->second.waypoints;
			waypointTimes   = temp->second.times;
			type            = temp->second.type;
			
			if(type == absolute)
			{
				return this->robot->move_object(objectWaypoints,waypointTimes);
			}
			else // type == relative
			{
				// Transform relative poses to absolute poses
				
				std::vector<Eigen::Isometry3d> newObjectPoints;
				
				newObjectPoints.push_back(this->robot->object_pose()*objectWaypoints[0]); // Transform pose of first waypoint relative to current object pose
				
				for(int i = 1; i < objectWaypoints.size(); i++)
				{
					newObjectPoints.push_back(newObjectPoints[i-1]*objectWaypoints[i]); // Transform pose of every subsequent waypoint relative to previous
				}
				return this->robot->move_object(newObjectPoints,waypointTimes);
			}
		}

		/**
		 * Move the joints according to a prescribed action.
		 * @param actionName The name of the joint space action listed in the relevant config file.
		 * @return Returns true if there were no problems.
		 */
		bool perform_joint_space_action(const std::string& actionName)
		{
			if(this->robot->is_grasping())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] perform_joint_space_action(): "
				          << "The robot is currently grasping something. "
				          << "You need to call 'deactivate_grasp()' to release it.\n";
				          
				return false;
			}
			
			auto jointConfig = this->jointActionMap->find(actionName);
			
			if(jointConfig == this->jointActionMap->end())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_to_named_configuration(): "
				          << "Could not find a joint configuration named '"
				          << actionName << "' in the list.\n";
				
				return false;
			}
			else
			{
				return this->robot->move_to_positions(jointConfig->second.waypoints,
				                                      jointConfig->second.times);
				return true;
			}
		}

		/**
		 * Move the hands to the specified poses.
		 * @param leftPose The desired left hand pose as a 4x4 homogeneous transformation matrix.
		 * @param rightPose The desired right hand pose as a 4x4 homogeneous transformation matrix.
		 * @param time The time in which to reach the poses.
		 * @return Returns true if there were no problems.
		 */
		bool move_hands_to_pose(const yarp::sig::Matrix& leftPose,
		                        const yarp::sig::Matrix& rightPose,
		                        const double time)
		{
			if(this->robot->is_grasping())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_hands_to_pose(): "
				          << "The robot is currently grasping something. "
				          << "You need to call 'deactivate_grasp()' to release it.\n";
				          
				return false;
			}
			else if(leftPose.rows() != 4 or leftPose.cols() != 4)
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_hands_to_pose(): "
				          << "Expected a 4x4 matrix for the left hand pose, but "
				          << "it was " << leftPose.rows() << "x" << leftPose.cols() << ".\n";
				
				return false;
			}
			else if(rightPose.rows() != 4 or rightPose.cols() != 4)
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_hands_to_pose(): "
				          << "Expected a 4x4 matrix for the right hand pose, but "
				          << "it was " << rightPose.rows() << "x" << rightPose.cols() << ".\n";
				
				return false;
			}
			
			Eigen::Matrix4d temp;
			
			// Convert yarp::sig::Matrix to Eigen::Isometry3d
			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++) temp(i,j) = leftPose[i][j];
			}
		
			Eigen::Isometry3d desiredLeft(temp);
			
			// Convert yarp::sig::Matrix to Eigen::Isometry3d
			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++) temp(i,j) = rightPose[i][j];
			}
			
			Eigen::Isometry3d desiredRight(temp);
			
			return this->robot->move_to_pose(desiredLeft, desiredRight, time);	
		}

		/**
		 * Move the joints to the given position.
		 * @param position A vector of desired joint positions.
		 * @param time The time to reach the desired positions.
		 * @return Returns true if there were no problems.
		 */
		bool move_joints_to_position(const std::vector<double>& position, const double time)
		{
			if(this->robot->is_grasping())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_joints_to_position(): "
				          << "The robot is currently holding something. "
				          << "You need to call 'deactivate_grasp()' to release it.\n";

				return false;
			}
			else	return this->robot->move_to_position(Eigen::VectorXd::Map(&position[0], position.size()), time);
		}

		/**
		 * Move a grasped object to a desired pose.
		 * @param pose The desired pose for the object as a 4x4 homogeneous transformation matrix.
		 * @param time The time to reach said pose.
		 * @return Returns true if there were no problems.
		 */
		bool move_object_to_pose(const yarp::sig::Matrix& pose, const double time)
		{
			if(not this->robot->is_grasping())
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_object_to_pose(): "
				          << "The robot is not currently holding anything. "
				          << "You need to call 'activate_grasp()' first.\n";
				          
				return false;
			}
			else if(pose.rows() != 4 or pose.cols() != 4)
			{
				std::cerr << "[ERROR] [CONTROL SERVER] move_object_to_pose(): "
				          << "Expected a 4x4 matrix for the pose argument, but it was "
				          << pose.rows() << "x" << pose.cols() << ".\n";
				
				return false;
			}
			else
			{
				std::cout << "[ERROR] [CONTROL SERVER] move_object_to_pose(): "
				          << "This function has not been completed yet...\n";
				return false;
			}
		}

		/**
		 * Deactivates grasping mode so the hands can move independently.
		 */
		bool release()
		{
			return this->robot->release_grasp();
		}

		/**
		 * Immediately stops the robot moving.
		 */
		void stop() { this->robot->halt(); }
		
		/**
		 * Deactivates the control server.
		 */
		void shut_down() { this->serverActive = false; }	
		
		///////////////////// Not defined in CommandInterface.h ///////////////////////////
		bool is_active() const { return this->serverActive; }
	    
	private:
		bool serverActive = true;                                                           ///< Used to shut down the server.
		
		BimanualControl *robot;                                                             ///< Pointer to the robot object being controlled.
		
		std::map<std::string, JointTrajectory> *jointActionMap;                             ///< Map of prescribed joint configurations

		std::map<std::string, CartesianMotion> *leftHandMap;                                ///< Pointer to map of left hand actions
		
		std::map<std::string, CartesianMotion> *rightHandMap;                               ///< Pointer to map of right hand actions
		
		std::map<std::string, CartesianMotion> *graspActionMap;                             ///< Pointer to map of grasping actions
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                             MAIN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	std::string errorMessage = "[ERROR] [CONTROL SERVER] ";
	
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 5)
	{
		std::cerr << errorMessage << "Port name, path to URDF, and path to config file are required. "
		          << "Usage: ./command_server /serverPortName /robotPortName /path/to/model.urdf /path/to/config.ini\n";
		
		return 1;                                                                           // Close with error
	}

	std::string serverPortName  = argv[1];
	std::string robotPortPrefix = argv[2];                                                      // Get the port names
	std::string pathToURDF      = argv[3];                                                      // Get the file path
	std::string pathToConfig    = argv[4];                                                      // Path to the configuration file
	
	// Generate port list prefixes
	std::vector<std::string> portList;
	portList.push_back(robotPortPrefix + "/torso");
	portList.push_back(robotPortPrefix + "/left_arm");
	portList.push_back(robotPortPrefix + "/right_arm");
	
	yarp::os::Property parameter; parameter.fromConfigFile(pathToConfig);                       // Load the properties from the config file
	
	try // to start up the robot
	{
		// Get the joint list from the config file
		yarp::os::Bottle *bottle; bottle = parameter.find("joint_names").asList();
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list of joint names was specified in " << pathToConfig << ".\n";
			return 1;
		}
		
		std::vector<std::string> jointNames = string_from_bottle(bottle);                   // Function specified in Utils.h

		// Load the joint-space actions from the config file
		std::map<std::string, JointTrajectory> jointActionMap;                              // NOTE: JointTrajectory is a data structure in Utils.H
		
		bottle->clear(); bottle = &parameter.findGroup("JOINT_SPACE_ACTIONS");
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No group called JOINT_SPACE_ACTIONS could be found in "
			          << pathToConfig << ".\n";
			
			return 1;
		}
		if(not load_joint_configurations(bottle,jointActionMap)) return 1;
		
		// Load the left hand actions from the config file
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("names").asList(); // Find all the listed names
		
		std::vector<std::string> nameList = string_from_bottle(bottle);                     // Convert to vector of strings
		
		std::map<std::string, CartesianMotion> leftHandMap;                                 // Create the map so we can search it later
		
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("left").asList();  // Get all the actions listed for the left hand
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list called 'left' in the CARTESIAN_ACTIONS group "
			                          << "could be found in " << pathToConfig << ".\n";
			return 1;
		}
		
		if(not load_cartesian_trajectories(bottle,nameList,leftHandMap)) return 1;          // Try and put the named actions together in the map
		
		// Load the right hand actions from the config file
		std::map<std::string, CartesianMotion> rightHandMap;
		
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("right").asList(); // Find all the right hand actions
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list called 'right' in the CARTESIAN_ACTIONS group "
			                          << "could be found in " << pathToConfig << ".\n";
			return 1;
		}
		
		if(not load_cartesian_trajectories(bottle,nameList,rightHandMap)) return 1;         // Put them in to the map
		
		// Load the grasp actions from the config file
		std::map<std::string, CartesianMotion> graspActionMap;                              // Create map object
		
		bottle->clear(); bottle = &parameter.findGroup("GRASP_ACTIONS");                    // Find grasp actions in config file
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "Could not find the the group called GRASP_ACTIONS "
			                          << " in " << pathToConfig << ".\n";
			return 1;
		}
		
		nameList = string_from_bottle(bottle->find("names").asList());                      // Get all the names
		
		if(not load_cartesian_trajectories(bottle,nameList,graspActionMap)) return 1;       // Load actions in to map

		BimanualControl robot(pathToURDF, jointNames, portList);                            // Start up the robot
		
		// Turn on/off motor control
		bool active = parameter.find("sending_motor_control").asBool();
		robot.motor_control(active);
		
		// Set the gains for the tasks
		double cartesian = parameter.findGroup("GAINS").find("cartesian").asFloat64();
		double redundancy = parameter.findGroup("GAINS").find("redundancy").asFloat64();
		
		if(not robot.set_control_gains(cartesian,redundancy)) return 1;
		
		// Set the singularity avoidance parameters
		double limit  = parameter.findGroup("SINGULARITY_AVOIDANCE").find("limit").asFloat64();
		double scalar = parameter.findGroup("SINGULARITY_AVOIDANCE").find("scalar").asFloat64();
		if(not robot.set_singularity_avoidance_params(scalar,limit)) return 1;
		
		// Set the QP solver parameters
		std::string method = parameter.findGroup("QP_SOLVER").find("method").asString();
		double barrierScalar = parameter.findGroup("QP_SOLVER").find("scalar").asFloat64();
		double tolerance = parameter.findGroup("QP_SOLVER").find("tolerance").asFloat64();
		
		if(method == "dual") robot.use_dual();                                              // For the QP Solver
				
		if(not robot.set_barrier_scalar(barrierScalar)
		or not robot.set_tolerance(tolerance)) return 1;
		
		// Establish communication over YARP
		yarp::os::Network yarp;
		yarp::os::Port port;
		ControlServer controlServer(&robot,
		                            &jointActionMap,
		                            &leftHandMap,
		                            &rightHandMap,
		                            &graspActionMap);  // Create command server
		                            
		controlServer.yarp().attachAsServer(port);
		
		if(not port.open(serverPortName))
		{
			std::cerr << errorMessage << "Could not open port /controlServer.\n";
			return 1;
		}
		
		while(controlServer.is_active())
		{
			std::cout << "\nWorker bees can leave.\n";
			yarp::os::Time::delay(5);
			std::cout << "Even drones can fly away.\n";
			yarp::os::Time::delay(5);
			std::cout << "The Queen is their slave.\n";
			yarp::os::Time::delay(20);
		}
		
		std::cout << "[INFO] [CONTROL SERVER] Shutting down. Arrivederci.\n";
		
		port.close();
		
		robot.close();
		
		return 0;                                                                           // No problems with main
	}
	catch(std::exception &exception)
	{
		std::cerr << errorMessage << "There was a problem with initialization.\n";
		          
		std::cout << exception.what() << std::endl;                                         // Inform the user
		
		return 1;                                                                           // Close with error
	}
	
	return 0;
}
