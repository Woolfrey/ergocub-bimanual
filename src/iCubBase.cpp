#include <iCubBase.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCubBase::iCubBase(const std::string &pathToURDF,
                   const std::vector<std::string> &jointList,
                   const std::vector<std::string> &portList,
                   const std::string              &robotModel)
                   :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointList, portList),                                             // Open communication with joint motors
                   _robotModel(robotModel),                                                         // iCub2, iCub3, ergoCub
                   q(Eigen::VectorXd::Zero(this->numJoints)),                                       // Set the size of the position vector
                   qdot(Eigen::VectorXd::Zero(this->numJoints)),                                    // Set the size of the velocity vector
                   J(Eigen::MatrixXd::Zero(12,this->numJoints)),                                    // Set the size of the Jacobian matrix
                   M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                       // Set the size of the inertia matrix
                   invM(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),                    // Set the size of the inverse inertia
                   desiredPosition(Eigen::VectorXd::Zero(this->numJoints))                          // Desired configuration when running Cartesian control
{
	iDynTree::ModelLoader loader;
	
	std::string message = "[ERROR] [ICUB BASE] Constructor: ";
	
	if(not loader.loadReducedModelFromFile(pathToURDF, jointList, "urdf"))
	{
		message += "Could not load model from the path " + pathToURDF + ".";
		throw std::runtime_error(message);
	}
	else
	{
		iDynTree::Model temp = loader.model();
		
		// Add custom hand frames and base/torso pose based on the model
		if(this->_robotModel == "iCub2")
		{
			temp.addAdditionalFrameToLink("l_hand", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,0.0),
			                                                  iDynTree::Position(0.05765, -0.00556, 0.01369)));
		
			temp.addAdditionalFrameToLink("r_hand", "right",
						      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,M_PI),
						                          iDynTree::Position(-0.05765, -0.00556, 0.01369)));

			this->basePose = iDynTree::Transform(iDynTree::Rotation::RPY(0,0,-M_PI),
			                                     iDynTree::Position(0,0,0));
		}
		else if(this->_robotModel == "iCub3")
		{
			throw std::runtime_error(message + "Hand transforms for iCub3 have not been programmed yet!");
		}
		else if(this->_robotModel == "ergoCub")
		{
			temp.addAdditionalFrameToLink("l_hand_palm", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
			                                                  iDynTree::Position(0, -0.02, -0.05)));
			                                                  
                	temp.addAdditionalFrameToLink("r_hand_palm", "right",
                				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
                				                          iDynTree::Position(0, 0.02, -0.05)));
			
			this->basePose = iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),
			                                     iDynTree::Position(0,0,0));
		}
		else
		{	
			message += "Expected 'iCub2', 'iCub3' or 'ergoCub' for the robot model argument, "
			                "but your input was '" + robotModel + "'.";
			                            
			throw std::invalid_argument(message);
		}
		
		// Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			message += "Could not generate iDynTree::KinDynComputations object from the model "
			              + loader.model().toString() + ".";

			throw std::runtime_error(message);
		}
		else
		{
			// Resize vectors and matrices based on number of joints
			this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control		
			
			this->generalForces.resize(this->computer.model());
				
			// Set the static parts of the grasp matrices
			
			// G = [    I    0     I    0 ]
			//     [ S(left) I S(right) I ]
			this->G.block(0,0,3,3).setIdentity();
			this->G.block(0,3,3,3).setZero();
			this->G.block(0,6,3,3).setIdentity();
			this->G.block(0,9,3,3).setZero();
			this->G.block(3,3,3,3).setIdentity();
			this->G.block(3,9,3,3).setIdentity();
			
			// C = [  I  -S(left) -I  S(right) ]
			//     [  0      I     0     -I    ]
			C.block(0,0,3,3).setIdentity();
			C.block(0,6,3,3) = -C.block(0,0,3,3);
			C.block(3,0,3,3).setZero();
			C.block(3,3,3,3).setIdentity();
			C.block(3,6,3,3).setZero();
			C.block(3,9,3,3) = -C.block(0,0,3,3);
			
			if(not update_state()) throw std::runtime_error(message + "Unable to read initial joint state from the encoders.");
			
			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from " << pathToURDF << ".\n";
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders(this->q, this->qdot))
	{		
		// Put data in iDynTree class to compute inverse dynamics
		// (there is probably a smarter way but I keep getting errors otherwise)
		iDynTree::VectorDynSize tempPosition(this->numJoints);
		iDynTree::VectorDynSize tempVelocity(this->numJoints);
		for(int i = 0; i < this->numJoints; i++)
		{
			tempPosition(i) = this->q(i);
			tempVelocity(i) = this->qdot(i);
		}

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->computer.setRobotState(this->basePose,
		                                tempPosition,
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
			Eigen::MatrixXd temp(6,6+this->numJoints);                                  // Temporary storage
			
			this->computer.getFrameFreeFloatingJacobian("left",temp);                   // Compute left hand Jacobian
			this->Jleft = temp.block(0,6,6,this->numJoints);                            // Remove floating base component
			this->J.block(0,0,6,this->numJoints) = this->Jleft;                         // Assign to combined matrix
			
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->Jright = temp.block(0,6,6,this->numJoints);                           // Remove floating base component
			this->J.block(6,0,6,this->numJoints) = this->Jright;                        // Assign to larger matrix
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute inertia matrix for joints & base
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			this->invM = this->M.partialPivLu().inverse();                              // We will need the inverse late
			
			// Compute Coriolis and gravity torques
			this->computer.generalizedBiasForces(this->generalForces);
			this->coriolisAndGravity = iDynTree::toEigen(this->generalForces.jointTorques());	
			
			// Update hand poses
			this->leftPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("left"));
			this->rightPose = iDynTree_to_Eigen(this->computer.getWorldTransform("right"));
			
			// Update the grasp and constraint matrices
			if(this->isGrasping)
			{
				// Assume the payload is rigidly attached to the left hand
				this->payload.update_state(this->leftPose, iDynTree::toEigen(this->computer.getFrameVel("left"))); 

				// G = [    I    0     I    0 ]
				//     [ S(left) I S(right) I ]
				
				// C = [  I  -S(left)  -I  S(right) ]
				//     [  0      I      0    -I     ]
				
				Eigen::Matrix<double,3,3> S;                                        // Skew symmetric matrix
				
				// Left hand component
				Eigen::Vector3d r = this->leftPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0 ;
				
				this->G.block(3,0,3,3) =  S;
				this->C.block(0,3,3,3) =  S;
				
				// Right hand component
				r = this->rightPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0;
				     
				this->G.block(3,6,3,3) = S;
				this->C.block(0,9,3,3) =-S;
			}
			
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [ICUB BASE] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [ICUB BASE] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Convert Eigen::Isometry3d to iDynTree::Transform                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Transform iCubBase::Eigen_to_iDynTree(const Eigen::Isometry3d &T)
{
	Eigen::Matrix<double,3,3> R = T.rotation();
	Eigen::Vector3d           p = T.translation();
	
	return iDynTree::Transform(iDynTree::Rotation(R),
	                           iDynTree::Position(p(0),p(1),p(2)));
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d iCubBase::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_position(const Eigen::VectorXd &position,
                                const double &time)
{
	if(position.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this robot has " << this->numJoints << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<Eigen::VectorXd> target; target.push_back(position);                    // Insert in to std::vector to pass onward
		std::vector<double> times; times.push_back(time);                                   // Time in which to reach the target position
		return move_to_positions(target,times);                                             // Call "main" function
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Stop the robot immediately                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->isFinished = true;
	send_joint_commands(this->q);                                                               // Hold current joint positions
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_positions(const std::vector<Eigen::VectorXd> &positions,
                                 const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [ICUB BASE] move_to_positions(): "
		          << "Position array had " << positions.size() << " waypoints, "
		          << "but the time array had " << times.size() << " elements!" << std::endl;

		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control thread that might be running
		this->controlSpace = joint;                                                         // Switch to joint control mode
		int m = positions.size() + 1;                                                       // We need to add 1 extra waypoint for the start
		iDynTree::VectorDynSize waypoint(m);                                                // All the waypoints for a single joint
		iDynTree::VectorDynSize t(m);                                                       // Times to reach the waypoints
		
		for(int i = 0; i < this->numJoints; i++)                                            // For the ith joint...
		{
			for(int j = 0; j < m; j++)                                                  // ... and jth waypoint
			{
				if(j == 0)
				{
					waypoint[j] = this->q[i];                                   // Current position is start point
					t[j] = 0.0;                                                 // Start immediately
				}
				else
				{
					double target = positions[j-1][i];                          // Get the jth target for the ith joint
					
					     if(target < this->positionLimit[i][0]) target = this->positionLimit[i][0] + 0.001;
					else if(target > this->positionLimit[i][1]) target = this->positionLimit[i][1] - 0.001;
					
					waypoint[j] = target;                                       // Assign the target for the jth waypoint
					
					t[j] = times[j-1];                                          // Add on subsequent time data
					
					if(t[j] <= t[j-1])
					{
						std::cerr << "[ERROR] [iCUB BASE] move_to_positions: "
						          << "Times must be in ascending order. Waypoint "
						          << j-1 << " had a time of " << t[j-1] << " and "
						          << "waypoint " << j << " had a time of " << t[j] << ".\n";
						
						return false;
					}
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [ICUB BASE] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}	
			else this->jointTrajectory[i].setInitialConditions(this->qdot[i],0.0);      // Use the current joint velocity
		}
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // Start the control thread
		
		return true;                                                                        // Success
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move each hand to a desired pose                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_pose(const Eigen::Isometry3d &desiredLeft,
                            const Eigen::Isometry3d &desiredRight,
                            const double &time)
{
	// Put them in to std::vector objects and pass onward
	std::vector<Eigen::Isometry3d> leftPoses(1,desiredLeft);
	std::vector<Eigen::Isometry3d> rightPoses(1,desiredRight);
	std::vector<double> times(1,time);
	
	return move_to_poses(leftPoses,rightPoses,times);                                           // Call full function
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move both hands through multiple poses                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_poses(const std::vector<Eigen::Isometry3d> &left,
                             const std::vector<Eigen::Isometry3d> &right,
                             const std::vector<double> &times)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlSpace = cartesian;                                                             // Switch to Cartesian control mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0.0);                                                    // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for each hand
	std::vector<Eigen::Isometry3d> leftPoints; leftPoints.push_back(this->leftPose);            // First waypoint is current pose
	leftPoints.insert(leftPoints.end(),left.begin(),left.end());
	
	std::vector<Eigen::Isometry3d> rightPoints; rightPoints.push_back(this->rightPose);
	rightPoints.insert(rightPoints.end(), right.begin(), right.end());
	
	try
	{
		Eigen::Matrix<double,6,1> twist = iDynTree::toEigen(this->computer.getFrameVel("left"));
		
		this->leftTrajectory = CartesianTrajectory(leftPoints,t,twist);                     // Assign new trajectory for left hand
		
		twist = iDynTree::toEigen(this->computer.getFrameVel("right"));
		
		this->rightTrajectory = CartesianTrajectory(rightPoints,t, twist);                  // Assign new trajectory for right hand
		
		this->endTime = times.back();                                                       // For checking when done
		
		start();                                                                            // Go to threadInit();
		
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_poses(): "
		          << "Unable to set new Cartesian trajectories.\n";
		
		std::cout << exception.what() << std::endl;

		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Grasp an object with two hands                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::grasp_object()
{
	if(this->isGrasping)
	{
		std::cout << "[ERROR] [ICUB BASE] grasp_object(): "
		          << "Already grasping an object! "
		          << "Need to let go with release_object() before grasping again.\n";
		          
		return false;
	}
	else
	{		
		this->isGrasping = true;                                                            // Set grasp constraint
		
		this->desiredLeft2Right = this->leftPose.inverse()*this->rightPose;
		
		double graspWidth = (this->leftPose.translation() - this->rightPose.translation()).norm(); // Distance between the hands
		
		Eigen::Isometry3d localPose(Eigen::Translation3d(0,-graspWidth/2,0));               // Negative y-axis of left hand, half the distance between hands
		
		this->payload = Payload(localPose);                                                 // Set the payload
		
		if(update_state()) return move_object(this->payload.pose(), 1.0);                   // Update pose, grasp constraints, activate control
		else
		{
			std::cout << "[ERROR] [ICUB BASE] grasp_object(): Something went wrong.\n";
			          
			return false;
		}
	}
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Release an object                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::release_object()
{
	if(this->isGrasping)
	{
		this->isGrasping = false;                                                          // Deactivate grasp constraints
		
		move_to_pose(this->leftPose,this->rightPose,1.0);                                  // Maintain current hand poses		
		
		return true;
	}
	else
	{
		std::cout << "[INFO] [ICUB BASE] release_object(): "
		          << "I'm not grasping anything!" << std::endl;
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Move the box to a given pose                                      //          
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const Eigen::Isometry3d &pose,
                           const double &time)
{
	if(not this->isGrasping)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "I am not grasping anything!\n";
		
		return false;
	}		         
	else if(time < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "Time of " << time << " cannot be negative!\n";
		          
		return false;
	}
	else
	{
		// Insert in to std::vector objects and pass on to spline generator
		std::vector<Eigen::Isometry3d> poses;
		poses.push_back(pose);
		
		std::vector<double> times;
		times.push_back(time);
		
		return move_object(poses,times);                                                    // Pass onward for spline generation
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Move the box through multiple poses                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const std::vector<Eigen::Isometry3d> &poses,
                           const std::vector<double> &times)
{
	if( isRunning() ) stop();                                                                   // Stop any control threads that are running
	
	this->controlSpace = cartesian;                                                             // Ensure that we are running in Cartesian mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0);                                                      // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for the object
	std::vector<Eigen::Isometry3d> waypoints; waypoints.push_back( this->payload.pose() );      // First waypoint is current pose
	waypoints.insert( waypoints.end(), poses.begin(), poses.end() );                            // Add on additional waypoints to the end
	
	try
	{

		this->payloadTrajectory = CartesianTrajectory(waypoints, t,this->payload.twist());  // Create new trajectory to follow
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // go immediately to threadInit()
		
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "Could not assign a new trajectory for the object.\n";
		          
		std::cout << exception.what() << std::endl;
		
		return false;       
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> iCubBase::pose_error(const Eigen::Isometry3d &desired,
                                               const Eigen::Isometry3d &actual)
{
	Eigen::Matrix<double,6,1> error;                                                            // Value to be computed
	
	error.block(0,0,3,1) = desired.translation() - actual.translation();                        // Position / translation error
	
	error.block(3,0,3,1) = angle_axis(desired.rotation()*(actual.rotation().transpose()));      // Get angle*axis representation of Rd*Ra'
	
	return error;
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Set the Cartesian gains                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_cartesian_gains(const double &proportional, const double &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_cartesian_gains(): "
		          << "Gains must be positive, but your inputs were " << proportional
		          << " and " << derivative << ".\n";
		
		return false;
	}
	else
	{
		this->K = proportional*this->gainTemplate;
		this->D = derivative*this->gainTemplate;

		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the joint gains                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_joint_gains(): "
		          << "Gains must be positive, but your inputs were " << proportional
		          << " and " << derivative << ".\n";
		          
		return false;
	}
	else
	{
		this->kp = proportional;
		this->kd = derivative;
		
		return true;
	}
}
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Return the pose of a given hand                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d iCubBase::hand_pose(const std::string &which)
{
	     if(which == "left")  return this->leftPose;
	else if(which == "right") return this->rightPose;
	else throw std::invalid_argument("[ERROR] [iCUB BASE] hand_pose(): Expected 'left' or 'right' but the argument was '"+which+"'.");
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Set the desired joint position when executing Cartesian control               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_desired_joint_position(const Eigen::VectorXd &position)
{
	if(position.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [iCUB BASE] set_desired_joint_position(): "
		          << "This robot has " << this->numJoints << " joints, but the argument "
		          << "had " << position.size() << " elements.\n";
		
		return false;
	}
	else
	{
		this->desiredPosition = position;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Set the parameters for singularity avoidance (i.e. damped least squares)        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_singularity_avoidance_params(const double &_maxDamping, const double &_threshold)
{
	if(_maxDamping <= 0 or _threshold <= 0)
	{
		std::cerr << "[ERROR] [iCUB BASE] set_singularity_avoidance_params(): "
		          << "Arguments must be positive, but the damping argument was "
		          << _maxDamping << ", and the threshold argument was " << _threshold << ".\n";
		
		return false;
	}
	else
	{
		this->maxDamping = _maxDamping;
		this->threshold = _threshold;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Decompose a rotation matrix in to its angle*axis representation                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d iCubBase::angle_axis(const Eigen::Matrix3d &R)
{
	double ratio = std::min( (R(0,0) + R(1,1) + R(2,2) - 1)/2.0 , 1.0 );                        // Rounding error can cause ratio > 1.0000000

	double angle = acos(ratio);

	if(abs(angle) < 1e-05)
	{
		return Eigen::Vector3d::Zero();                                                     // Angle is small so axis is trivial
	}                                  
	else
	{
		double scalar = angle/(2*sin(angle));
		
		return Eigen::Vector3d(scalar*(R(2,1)-R(1,2)),
		                       scalar*(R(0,2)-R(2,0)),
		                       scalar*(R(1,0)-R(0,1)));
	}
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Get the partial derivative of a Jacobian w.r.t a given joint                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd iCubBase::partial_derivative(const Eigen::MatrixXd &J, const unsigned int &jointNum)
{
	if(J.rows() != 6)
	{
		throw std::invalid_argument("[ERROR] [iCUB BASE] partial_derivative(): Expected the Jacobian argument to have 6 rows, but it had " + std::to_string(this->J.rows()) + ".");
	}
	else if(jointNum > this->numJoints - 1)
	{
		throw std::invalid_argument("[ERROR] [iCUB BASE] partial_derivative(): Cannot compute the partial derivative for joint " + std::to_string(jointNum) + " as this model only has " + std::to_string(this->numJoints) + " joints.");
	}
	
	Eigen::MatrixXd dJ(6,this->numJoints); dJ.setZero();
	
	for(int i = 0; i < this->numJoints; i++)
	{
		if (jointNum < i)
		{
			// a_j x (a_i x a_i)
			dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
			dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
			dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);

			// a_j x a_i
			dJ(3,i) = J(4,jointNum)*J(5,i) - J(5,jointNum)*J(4,i);
			dJ(4,i) = J(5,jointNum)*J(3,i) - J(3,jointNum)*J(5,i);
			dJ(5,i) = J(3,jointNum)*J(4,i) - J(4,jointNum)*J(3,i);
		}
		else
		{
			// a_i x (a_j x a_j)
			dJ(0,i) = J(4,i)*J(2,jointNum) - J(5,i)*J(1,jointNum);
			dJ(1,i) = J(5,i)*J(0,jointNum) - J(3,i)*J(2,jointNum);
			dJ(2,i) = J(3,i)*J(1,jointNum) - J(4,i)*J(0,jointNum);
		}
	}
	
	return dJ;
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Set the redundant task                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_redundant_task(const std::string &task, const double &scalar)
{
	this->kr = scalar;
	
	if(task == "default" or task == "singularity_avoidance")
	{
		this->redundantTask = singularityAvoidance;
		return true;
	}
	else if(task == "set_point")
	{
		this->redundantTask = setPoint;
		return true;
	}
	else
	{
		std::cerr << "[ERROR] [iCUB BASE] set_redundant_task(): "
		          << "You input " << task << " but current can only do "
		          << "'singularity_avoidance' / 'default' and 'set_point'.\n";
		return false;
	}
}		      

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the designated redundant task                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd iCubBase::redundant_task()
{
	Eigen::VectorXd task(this->numJoints); task.setZero();                                      // Value to be returned
	
	switch(this->redundantTask)
	{
		case setPoint:
		{
			for(int i = 0; i < this->numJoints; i++)
			{
				task(i) = this->kr*(this->desiredPosition(i) - this->q(i));         // Try to maintain desired pose
			}
			break;
		}
		default: // manipulability
		{	
			// Values used in this scope
			Eigen::Matrix<double,6,6> JJt_left = (this->Jleft*this->Jleft.transpose()).partialPivLu().inverse();
			Eigen::Matrix<double,6,6> JJt_right = (this->Jright*this->Jright.transpose()).partialPivLu().inverse();
			
			// We have to do this weird backwards iteration since the
			// torso on the ergoCub is broken
			
			for(int i = this->numJoints-1; i >= 0; i--)
			{
				if(i >= this->numJoints - 7)                                         // Right arm
				{
					task(i) = this->kr * this->manipulability
					        * (JJt_right * partial_derivative(this->Jright,i) * this->Jright.transpose()).trace();
				}
				else if(i >= this->numJoints - 14)                                   // Left arm
				{
					task(i) = this->kr * this->manipulability
					        * (JJt_left * partial_derivative(this->Jleft,i) * this->Jleft.transpose()).trace();
				}
				else    task(i) = -this->kp*this->q(i);                             // Torso
			}
		}
	}
	
	return task;
}
