#include <BimanualControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
BimanualControl::BimanualControl(const std::string              &pathToURDF,
                                 const std::vector<std::string> &jointList,
                                 const std::vector<std::string> &portList)
                                 :
                                 MotorControl(jointList,portList),
                                 yarp::os::PeriodicThread(0.01),                                    // Create thread to run at 100Hz
                                 J(Eigen::MatrixXd::Zero(12,this->numJoints)),                      // Resize Jacobian
                                 M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints)),
                                 jointNames(jointList)
{	
	// Resize vectors based on number of joints
	this->jointPos.resize(this->numJoints);
	this->jointVel.resize(this->numJoints);
	this->jointRef.resize(this->numJoints);
	
	// Set up YARP ports for communication
	this->jointReferences.open("/jointReferences");
	this->jointTrackingError.open("/jointTrackingError");
	this->walkingModuleInterface.open("/bimanualUpperRefs");
	this->objectTrackingError.open("/objectTrackingError");
	this->constraintAdherence.open("/constraintAdherence");
	this->manipulabilityData.open("/manipulability");
	
	iDynTree::ModelLoader loader;
	
	if(not loader.loadReducedModelFromFile(pathToURDF, jointList, "urdf"))
	{
		throw std::runtime_error("[ERROR] [BIMANUAL CONTROL] Constructor: "
		                         "Could not load model from the path " + pathToURDF + ".");
	}
	else
	{
		iDynTree::Model temp = loader.model();

		temp.addAdditionalFrameToLink("l_hand_palm", "left",
		                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
		                                                  iDynTree::Position(0, -0.02, -0.05)));
		                                                  
        	temp.addAdditionalFrameToLink("r_hand_palm", "right",
        				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
        				                          iDynTree::Position(0, 0.02, -0.05)));
		
		this->basePose = iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),
		                                     iDynTree::Position(0,0,0));
		
		// Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			throw std::runtime_error("[ERROR] [BIMANUAL CONTROL] Constructor: "
			                         "Could not generate iDynTree::KinDynComputations object from the model " + loader.model().toString() + ".");
		}
		else
		{
			this->jointTrajectory.resize(this->numJoints);                              // Trajectory for joint motion control
			
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
			
			// Set up the motor controller
			
			if(not MotorControl::read_encoders(this->jointRef, this->jointVel))         // Put the current joint values as the start ref position
			{
				throw std::runtime_error("[ERROR] [BIMANUAL CONTROL] Constructor: "
			                                 "Unable to read initial joint state from the encoders.");
			}
			else if(not update_state())
			{
				throw std::runtime_error("[ERROR] [BIMANUAL CONTROL] Constructor: "
				                         "Unable to update the kinematics and dynamics for some reason.");
			}
			                                                
			std::cout << "[INFO] [BIMANUAL CONTROL] Successfully created iDynTree model from " << pathToURDF << ".\n";
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::update_state()
{
	if(MotorControl::read_encoders(this->jointPos, this->jointVel))
	{		
		iDynTree::VectorDynSize tempPosition(this->jointRef);                               // Can't use real values when running robot in position mode...
		iDynTree::VectorDynSize tempVelocity(this->jointVel);

		// Put them in to the iDynTree::KinDynComputations class to solve
		if(this->computer.setRobotState(this->basePose,
		                                tempPosition,
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                tempVelocity,                                                                // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81})))                   // Direction of gravity
		{
			Eigen::MatrixXd temp(6,6+this->numJoints);                                  // Temporary storage
			
			// Left hand jacobian
			this->computer.getFrameFreeFloatingJacobian("left",temp);                   // Compute left hand Jacobian
			this->Jleft = temp.block(0,6,6,this->numJoints);                            // Remove floating base component
			this->J.block(0,0,6,this->numJoints) = this->Jleft;                         // Assign to combined matrix
			
			// Right hand jacobian
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->Jright = temp.block(0,6,6,this->numJoints);                           // Remove floating base component
			this->J.block(6,0,6,this->numJoints) = this->Jright;                        // Assign to larger matrix
			
			// Compute inertia matrix
			temp.resize(6+this->numJoints,6+this->numJoints);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute inertia matrix for joints & base
			this->M = temp.block(6,6,this->numJoints,this->numJoints);                  // Remove floating base
			
			// Update hand poses
			this->leftPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("left"));
			this->rightPose = iDynTree_to_Eigen(this->computer.getWorldTransform("right"));
			
			if(this->isGrasping)
			{
				this->objectPose = this->leftPose*this->leftHand2Object;            // Assume object is rigidly attached to left hand
			
				// G = [    I    0     I    0 ]
				//     [ S(left) I S(right) I ]
				
				// C = [  I  -S(left)  -I  S(right) ]
				//     [  0      I      0    -I     ]
				
				Eigen::Matrix<double,3,3> S;                                        // Skew symmetric matrix
				
				// Left hand component
				Eigen::Vector3d r = this->leftPose.translation() - this->objectPose.translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0 ;
				
				this->G.block(3,0,3,3) =  S;
				this->C.block(0,3,3,3) =  S;
				
				// Right hand component
				r = this->rightPose.translation() - this->objectPose.translation();
				
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
			std::cerr << "[ERROR] [BIMANUAL CONTROL] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Convert Eigen::Isometry3d to iDynTree::Transform                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Transform BimanualControl::Eigen_to_iDynTree(const Eigen::Isometry3d &T)
{
	Eigen::Matrix<double,3,3> R = T.rotation();
	Eigen::Vector3d           p = T.translation();
	
	return iDynTree::Transform(iDynTree::Rotation(R),
	                           iDynTree::Position(p(0),p(1),p(2)));
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d BimanualControl::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::move_to_position(const Eigen::VectorXd &position,
                                       const double &time)
{
	if(position.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_to_position(): "
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
void BimanualControl::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->isFinished = true;
	send_joint_commands(this->jointPos);                                                        // Hold current joint positions
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::move_to_positions(const std::vector<Eigen::VectorXd> &positions,
                                        const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [BIMANUAL CONTROL] move_to_positions(): "
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
					waypoint[j] = this->jointPos[i];                            // Current position is start point
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
						std::cerr << "[ERROR] [BIMANUAL CONTROL] move_to_positions: "
						          << "Times must be in ascending order. Waypoint "
						          << j-1 << " had a time of " << t[j-1] << " and "
						          << "waypoint " << j << " had a time of " << t[j] << ".\n";
						
						return false;
					}
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [BIMANUAL CONTROL] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}	
			else this->jointTrajectory[i].setInitialConditions(this->jointVel[i],0.0);  // Use the current joint velocity
		}
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // Start the control thread
		return true;                                                                        // Success
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move each hand to a desired pose                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::move_to_pose(const Eigen::Isometry3d &desiredLeft,
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
bool BimanualControl::move_to_poses(const std::vector<Eigen::Isometry3d> &left,
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
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_to_poses(): "
		          << "Unable to set new Cartesian trajectories.\n";
		
		std::cout << exception.what() << std::endl;

		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Move the box to a given pose                                      //          
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::move_object(const Eigen::Isometry3d &pose,
                                  const double &time)
{
	if(not this->isGrasping)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_object(): "
		          << "I am not grasping anything!\n";
		
		return false;
	}		         
	else if(time < 0)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_object(): "
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
bool BimanualControl::move_object(const std::vector<Eigen::Isometry3d> &poses,
                                  const std::vector<double> &times)
{
	if(not this->isGrasping)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_object(): "
		          << "Not current grasping anything. Did you forget to call 'activate_grasp()'?\n";
		          
		return false;
	}
	
	Eigen::Vector<double,6> leftHandTwist = iDynTree::toEigen(this->computer.getFrameVel("left"));
	Eigen::Vector3d angularVel = leftHandTwist.tail(3);
	
	Eigen::Vector<double,6> objectTwist;
	objectTwist.head(3) = leftHandTwist.head(3) + angularVel.cross(this->objectPose.translation() - this->leftPose.translation());
	objectTwist.tail(3) = angularVel;                                  
	
	if( isRunning() ) stop();                                                                   // Stop any control threads that are running
	
	this->controlSpace = cartesian;                                                             // Ensure that we are running in Cartesian mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0);                                                      // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for the object
	std::vector<Eigen::Isometry3d> waypoints; waypoints.push_back(this->objectPose);            // First waypoint is current pose
	waypoints.insert(waypoints.end(),poses.begin(),poses.end());                                // Add on additional waypoints
	
	try
	{
		this->objectTrajectory = CartesianTrajectory(waypoints, t, objectTwist);            // Create new trajectory to follow
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // go immediately to threadInit()
		
		return true;
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_object(): "
		          << "Could not assign a new trajectory for the object.\n";
		          
		std::cout << exception.what() << std::endl;
		
		return false;       
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> BimanualControl::pose_error(const Eigen::Isometry3d &desired,
                                                      const Eigen::Isometry3d &actual)
{
	Eigen::Matrix<double,6,1> error;                                                            // Value to be computed
	
	error.block(0,0,3,1) = desired.translation() - actual.translation();                        // Position / translation error

	error.block(3,0,3,1) = angle_axis(desired.rotation()*(actual.rotation().transpose()));      // Get angle*axis representation of Rd*Ra'
	
	return error;
}
 
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Return the pose of a given hand                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d BimanualControl::hand_pose(const std::string &which)
{
	     if(which == "left")  return this->leftPose;
	else if(which == "right") return this->rightPose;
	else throw std::invalid_argument("[ERROR] [BIMANUAL CONTROL] hand_pose(): Expected 'left' or 'right' but the argument was '"+which+"'.");
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Set the parameters for singularity avoidance (i.e. damped least squares)        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::set_singularity_avoidance_params(const double &scalar, const double &limit)
{
	if(scalar <= 0 or limit <= 0)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] set_singularity_limit(): "
		          << "Input arguments must be positive. "
		          << "Scalar argument was " << to_string(scalar) << ", and "
		          << "limit argument was " << to_string(limit) << ".\n";
		          
		return false;
	}

	else
	{
		this->barrierScalar        = scalar;
		this->manipulabilityLimit  = limit;
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Decompose a rotation matrix in to its angle*axis representation                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d BimanualControl::angle_axis(const Eigen::Matrix3d &R)
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
Eigen::MatrixXd BimanualControl::partial_derivative(const Eigen::MatrixXd &J, const unsigned int &jointNum)
{
	if(J.rows() != 6)
	{
		throw std::invalid_argument("[ERROR] [BIMANUAL CONTROL] partial_derivative(): Expected the Jacobian argument to have 6 rows, but it had " + std::to_string(this->J.rows()) + ".");
	}
	else if(jointNum > this->numJoints - 1)
	{
		throw std::invalid_argument("[ERROR] [BIMANUAL CONTROL] partial_derivative(): Cannot compute the partial derivative for joint " + std::to_string(jointNum) + " as this model only has " + std::to_string(this->numJoints) + " joints.");
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
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::threadInit()
{
	if(isRunning())
	{
		std::cout << "[ERROR] [BIMANUAL CONTROL] threadInit(): "
		          << "A control thread is still running!\n";
		          
		return false;
	}
	else
	{
		QPSolver::clear_last_solution();                                                    // Remove last solution
		
		this->isFinished = false;                                                           // New action started

		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		
		this->jointRef = this->jointPos;                                                    // Reference position = current position
		
		return true;                                                                        // jumps immediately to run()
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void BimanualControl::run()
{
	if(update_state())
	{
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of control
		
		if(elapsedTime > this->endTime) this->isFinished = true;                              
		
		if(this->controlSpace == joint)
		{		
			for(int i = 0; i < this->numJoints; i++) this->jointRef[i] = this->jointTrajectory[i].evaluatePoint(elapsedTime);
		}
		else // this->controlSpace == Cartesian
		{
			Eigen::VectorXd qdot(this->numJoints); qdot.setZero();                      // We want to solve for this
		
			// Compute the desired hand velocities and convert
			// to a discrete time step		
		
			Eigen::Vector<double,12> xdot; xdot.setZero();
			Eigen::Isometry3d pose;
			Eigen::Vector<double,6> vel, acc;
			
			if(this->isGrasping)
			{
				this->objectTrajectory.get_state(pose,vel,acc,elapsedTime);         // Get the desired object state for the given time
				
				this->objectPoseError = pose_error(pose,this->objectPose);          // Save this so we can publish it later            
				
				xdot = this->G.transpose()*(vel + this->K*this->objectPoseError);
				
			}
			else
			{
				this->leftTrajectory.get_state(pose,vel,acc,elapsedTime);           // Desired state for the left hand

				xdot.head(6) = vel + this->K*pose_error(pose,this->leftPose);

				this->rightTrajectory.get_state(pose,vel,acc,elapsedTime);          // Desired state for the right hand
				
				xdot.tail(6) = vel + this->K*pose_error(pose,this->rightPose);
			}
	
			// Compute redundant task, joint limits
			
			Eigen::Matrix<double,6,6> JJTleft  = this->Jleft*this->Jleft.transpose();
			Eigen::Matrix<double,6,6> JJTright = this->Jright*this->Jright.transpose();
			
			Eigen::LDLT<Eigen::Matrix<double,6,6>> JJTleft_decomp(JJTleft);
			Eigen::LDLT<Eigen::Matrix<double,6,6>> JJTright_decomp(JJTright);
			
			double m_left  = sqrt(JJTleft.determinant());
			double m_right = sqrt(JJTright.determinant());
			
			this->manipulability = std::min(m_left, m_right);
			
			Eigen::VectorXd dmdq_left(this->numJoints);
			Eigen::VectorXd dmdq_right(this->numJoints);
			Eigen::VectorXd redundantTask(this->numJoints);
			Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);
			
			for(int i = 0; i < this->numJoints; i++)
			{
				compute_control_limits(lowerBound(i), upperBound(i), i);            // Instantaneous limits on joint motion
				
				// Compute gradient of manipulability for each arm
				if(i == 0)                                                          // First joint does nothing
				{
					dmdq_left(i)  = 0.0;
					dmdq_right(i) = 0.0;
				}
				else
				{			
					dmdq_left(i) = m_left   * (JJTleft_decomp.solve( partial_derivative(this->Jleft,i)*this->Jleft.transpose() )).trace();
					dmdq_right(i) = m_right * (JJTright_decomp.solve( partial_derivative(this->Jright,i)*this->Jright.transpose() )).trace();
				}
				
				// Formulate redundant task
				if(i < 3)	redundantTask(i) = -this->jointRef[i];              // Drive torso joints to zero
				else if(i < 10) redundantTask(i) = dmdq_left(i);                    // Increase manipulability of left arm
				else		redundantTask(i) = dmdq_right(i);                   // Increase manipulability of right arm
			}
			
			redundantTask *= this->redundantScalar;                                     // Scale to increase/decrease effect
			
			// Set up constraints for QP solver
			
			Eigen::MatrixXd B(2*this->numJoints+2,this->numJoints);
			B.block(0,0,this->numJoints,this->numJoints).setIdentity();
			B.block(this->numJoints,0,this->numJoints,this->numJoints) = -Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
			B.row(2*this->numJoints)   = -dmdq_left.transpose();
			B.row(2*this->numJoints+1) = -dmdq_right.transpose();
			
			Eigen::VectorXd z(2*this->numJoints+2);
			z.block(              0,0,this->numJoints,1) =  upperBound;
			z.block(this->numJoints,0,this->numJoints,1) = -lowerBound;
			
			if(this->manipulability > this->manipulabilityLimit)
			{
				z(2*this->numJoints)   = this->barrierScalar * (m_left  - this->manipulabilityLimit);
				z(2*this->numJoints+1) = this->barrierScalar * (m_right - this->manipulabilityLimit);
			}
			else
			{
				z(2*this->numJoints) = 0.0;
				z(2*this->numJoints+1) = 0.0;
			}
			
			// Need a start point for the QP solver
			Eigen::VectorXd startPoint = QPSolver::last_solution();
			if(startPoint.size() != this->numJoints) startPoint = 0.5*(lowerBound + upperBound);
			
			try
			{				
				qdot = QPSolver::constrained_least_squares(redundantTask, this->M, this->J, xdot, B, z, startPoint);
			}
			catch(const std::exception &exception)
			{
				std::cout << exception.what() << std::endl;
			}

			
			if(this->isGrasping) // Re-solve the QP problem subject to grasp constraints
			{	
			 	Eigen::MatrixXd Jc = this->C*this->J;                               // Constraint Jacobian
			 	
				try // Too easy lol ᕙ(▀̿̿ĺ̯̿̿▀̿ ̿) ᕗ
				{
					qdot = QPSolver::constrained_least_squares(qdot, this->M,  Jc, Eigen::VectorXd::Zero(6), B, z, qdot);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
				}
			}
			
			for(int i = 0; i < this->numJoints; i++) this->jointRef[i] += this->dt*qdot(i);        // Increment the reference position
		}


		if(this->motorControlActive and not send_joint_commands(this->jointRef))
		{
			std::cout << "[ERROR] [BIMANUAL CONTROL] Could not send joint commands for some reason.\n";
		}
		
		// Set up YARP ports to publish data
		yarp::sig::Vector &mu = this->manipulabilityData.prepare();
		yarp::sig::Vector &jointRefData   = this->jointReferences.prepare();
		yarp::sig::Vector &jointErrorData = this->jointTrackingError.prepare();
		WalkingControllers::YarpUtilities::HumanState &walkingModuleData = this->walkingModuleInterface.prepare();
		
		// Clear data to enable new inputs
		mu.clear();
		jointRefData.clear();
		jointErrorData.clear();
		walkingModuleData.jointNames.clear();
		walkingModuleData.positions.clear();
		
		// Input the data
		
		mu.push_back(this->manipulability);
		
		for(int i = 0; i < this->numJoints; i++)
		{
			jointRefData.clear();
			jointErrorData.clear();
			walkingModuleData.positions.push_back(this->jointRef[i]);
			walkingModuleData.jointNames.push_back(this->jointNames[i]);
		}
		
		// Write data to the port
		this->jointReferences.write();
		this->jointTrackingError.write();
		this->walkingModuleInterface.write();
		this->manipulabilityData.write();
		
		// This is for assessing constraints, etc
		if(this->isGrasping)
		{
			yarp::sig::Vector &constraintData = this->constraintAdherence.prepare();
			constraintData.clear();
			Eigen::Vector<double,6> temp = pose_error(this->leftPose, this->rightPose); // Get the pose error between the hands
			constraintData.push_back(temp.head(3).norm()*1000);                         // Position error in mm
			constraintData.push_back(temp.tail(3).norm()*180/3.141592);                 // Orientation error in deg
			
			this->constraintAdherence.write();
			
			yarp::sig::Vector &objectErrorData = this->objectTrackingError.prepare();     
			objectErrorData.clear();
			objectErrorData.push_back(this->objectPoseError.head(3).norm()*1000);
			objectErrorData.push_back(this->objectPoseError.tail(3).norm()*180/3.141592);
			
			this->objectTrackingError.write();
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void BimanualControl::threadRelease()
{
	this->jointRef = this->jointPos;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::compute_control_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->numJoints)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] compute_joint_limits(): "
		          << "Range of joint indices is 0 to " << this->numJoints - 1 << ", "
		          << "but you called for " << jointNum << ".\n";

		return false;
	}
	else
	{   		
		lower = std::max( (this->positionLimit[jointNum][0] - this->jointRef[jointNum])/this->dt, // Distance to lower limit
		                  -this->velocityLimit[jointNum]);                                        // Maximum (negative) speed
		                
		upper = std::min( (this->positionLimit[jointNum][1] - this->jointRef[jointNum])/this->dt, // Distance to upper limit
		                   this->velocityLimit[jointNum]);                                        // Maximum speed

		if(lower >= upper)
		{
			std::cerr << "[ERROR] [BIMANUAL CONTROL] compute_joint_limits(): "
				  << "Lower limit " << lower << " for joint " << jointNum << " is greater than upper limit " << upper << ". "
				  << "How did that happen???\n";
				  
			return false;
		}
		else	return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Put the robot in to 'grasp' mode to control an object with 2 hands                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::activate_grasp()
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
		
		double graspWidth = (this->leftPose.translation() - this->rightPose.translation()).norm(); // Distance between the hands
		
		this->leftHand2Object = Eigen::Isometry3d(Eigen::Translation3d(0,-graspWidth/2,0)); // Assume object is rigidly attached to left hand
		
		this->objectPose = this->leftPose*this->leftHand2Object;                            // Update the object pose
		 
		return move_object(this->objectPose,1.0);                                           // Hold object in current pose
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //             Release from 'grasp' mode so the hands can move independently again                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::release_grasp()
{
	if(this->isGrasping)
	{
		this->isGrasping = false;
		
		return move_to_pose(this->leftPose,this->rightPose,1.0);
	}
	else
	{
		std::cout << "[INFO] [BIMANUAL CONTROL] release_grasp(): "
		          << "Not grasping anything!\n";
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Set the gains for different control tasks                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BimanualControl::set_control_gains(const double &cartesian, const double &redundant)
{
	if(cartesian < 0)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] set_control_gains(): "
		          << "Cartesian gain was " << cartesian << " but it must be positive.\n";

		return false;
	}
	else
	{
		this->K = cartesian * this->gainTemplate;
		
		this->redundantScalar = redundant;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set boolean for sending commands to the motors or not                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void BimanualControl::motor_control(const bool &active)
{
	if(active)
	{
		this->motorControlActive = true;
	
		std::cout << "[INFO] [BIMANUAL CONTROL] motor_control(): "
		          << "This controller will take control of the joint motors.\n";
	}
	else
	{
		this->motorControlActive = false;
		
		std::cout << "[INFO] [BIMANUAL CONTROL] motor_control(): "
		          << "Joint commands will NOT be sent to the joint motors.\n";
	}
}
