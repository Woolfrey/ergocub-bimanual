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
                                 M(Eigen::MatrixXd::Zero(this->numJoints,this->numJoints))
{	
	// Resize vectors based on number of joints
	this->jointPos.resize(this->numJoints);
	this->jointVel.resize(this->numJoints);
	this->jointRef.resize(this->numJoints);
	
	// Set up YARP ports for communication
	this->desiredJointPos.open("/desiredJointPos");
	this->jointTrackingError.open("/jointTrackingError");
	
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
						
			if(not update_state()) throw std::runtime_error("[ERROR] [BIMANUAL CONTROL] Constructor: "
			                                                "Unable to read initial joint state from the encoders.");
			
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
		// Put data in iDynTree class to compute inverse dynamics
		// (there is probably a smarter way but I keep getting errors otherwise)
		iDynTree::VectorDynSize tempPosition(this->numJoints);
		iDynTree::VectorDynSize tempVelocity(this->numJoints);
		for(int i = 0; i < this->numJoints; i++)
		{
			tempPosition(i) = this->jointPos[i];
			tempVelocity(i) = this->jointVel[i];
		}

		// Put them in to the iDynTree::KinDynComputations class to solve
		if(this->computer.setRobotState(this->basePose,
		                                tempPosition,
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                tempVelocity,                                       // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
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
	Eigen::Vector<double,6> leftHandTwist = iDynTree::toEigen(this->computer.getFrameVel("left"));
	Eigen::Vector3d angularVel = leftHandTwist.tail(3);
	
	std::cout << "\nHere is the left hand twist:\n";
	std::cout << leftHandTwist.transpose() << std::endl;
	
	Eigen::Vector<double,6> objectTwist;
	objectTwist.head(3) = leftHandTwist.head(3) + angularVel.cross(this->objectPose.translation() - this->leftPose.translation());
	objectTwist.tail(3) = angularVel;                                  
	
	std::cout << "\nHere is the object twist:\n";
	std::cout << objectTwist.transpose() << std::endl;
	
	if( isRunning() ) stop();                                                                   // Stop any control threads that are running
	
	this->controlSpace = cartesian;                                                             // Ensure that we are running in Cartesian mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0);                                                      // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for the object
	std::vector<Eigen::Isometry3d> waypoints; waypoints.push_back(this->objectPose);            // First waypoint is current pose
	waypoints.insert(waypoints.end(),poses.begin(),poses.end());                                // Add on additional waypoints
	
	std::cout << "\nSo far so good.\n";
	
	try
	{
		std::cout << "\nWe are in the try loop\n";
		
		this->payloadTrajectory = CartesianTrajectory(waypoints, t, objectTwist);           // Create new trajectory to follow
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] move_object(): "
		          << "Could not assign a new trajectory for the object.\n";
		          
		std::cout << exception.what() << std::endl;
		
		return false;       
	}
	
	this->endTime = times.back();                                                               // Assign the end time
	
	start();                                                                                    // go immediately to threadInit()
	
	std::cout << "\nSuccess!\n";
	
	return true;
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
			Eigen::VectorXd dq(this->numJoints); dq.setZero();                          // We want to solve for this		
		
			Eigen::VectorXd dx; dx.setZero(12);                                         // Discrete step for the change in hand poses
			
			if(this->isGrasping)
			{
				std::cout << "\nBefore trajectory\n";
				this->objectPose = this->payloadTrajectory.get_pose(elapsedTime);   // Desired pose of the object
				
				std::cout << "\nAfter trajectory\n";
				
				dx.head(6) = this->dt*pose_error(this->objectPose*object2Left, this->leftPose); // Error between desired left hand placement and actual
				dx.tail(6) = this->dt*pose_error(this->objectPose*object2Right, this->rightPose);
				
				std::cout << "\nHere is the desired hand motion:\n";
				std::cout << dx.transpose() << std::endl;
			}
			else
			{	
				dx.head(6) = this->dt*pose_error(this->leftTrajectory.get_pose(elapsedTime),  this->leftPose); // Difference between desired and actual pose		
				dx.tail(6) = this->dt*pose_error(this->rightTrajectory.get_pose(elapsedTime), this->rightPose); // Difference between desired and actual
			}
			
			dx *= this->cartesianScalar;                                                // THIS IS VERY IMPORTANT
			
			// Get the instantaneous limits on the joint motion
			Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);
			for(int i = 0; i < this->numJoints; i++) compute_control_limits(lowerBound(i),upperBound(i),i);
			
			// Compute the start point for the QP Solver
			Eigen::VectorXd startPoint = QPSolver::last_solution();
			
			if(startPoint.size() != this->numJoints)
			{
				startPoint = 0.5*(lowerBound + upperBound); // No solution, so start in the middle
			}
			
			double manipulability = sqrt((this->J*this->J.transpose()).determinant());  // Proximity to a singularity
			
			if(manipulability > this->manipulabilityLimit)  
			{
				// Compute the redundant task for singularity avoidance
				Eigen::VectorXd redundantTask(this->numJoints);
				
				Eigen::Matrix<double,6,6> JJt_left  = (this->Jleft*this->Jleft.transpose()).partialPivLu().inverse();
				Eigen::Matrix<double,6,6> JJt_right = (this->Jright*this->Jright.transpose()).partialPivLu().inverse();
				
				for(int i = 0; i < 7; i++)
				{
					// Torso joints
					if(i < 3) redundantTask(i) = -this->jointPos[i];            // Try and keep the torso upright
					
					// Left arm; offset by 3 for the torso joints
					redundantTask(i+3) = manipulability
						           * (JJt_left * partial_derivative(this->Jleft,i+3) * this->Jleft.transpose()).trace();
						           
					// Right arm; offset by 3+7 = 10 for torso, left arm joints
					redundantTask(i+10) = manipulability
						            * (JJt_right * partial_derivative(this->Jright,i+10) * this->Jright.transpose()).trace();					
				}
				
				redundantTask *= this->redundantScalar;                             // THIS IS VERY IMPORTANT
				
				try
				{				
					dq = QPSolver::constrained_least_squares(redundantTask, this->M, this->J, dx, lowerBound, upperBound, startPoint);
					
					std::cout << "\nThe solution error is: " << (dx - this->J*dq).norm() << std::endl;
				
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
					
					dq.setZero();                                               // Don't move!
				}
				
			}
			else // Near singular, assume dq = J'*dx to avoid inversion
			{
				std::cout << "\n[WARNING] Singular!\n";
				/*
				try
				{
					dq = QPSolver::constrained_least_squares(J.transpose()*dx, Eigen::MatrixXd::Identity(this->numJoints,this->numJoints), this->M, lowerBound, upperBound, startPoint);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
					
					dq.setZero();                                               // Don't move!
				}
				*/
			}
			
			for(int i = 0; i < this->numJoints; i++) this->jointRef[i] += dq(i);        // Increment the reference position
		}
		
		/*
		// Send data of YARP port
		yarp::sig::Vector &tempErrorVector   = this->jointTrackingError.prepare();
		yarp::sig::Vector &tempDesiredVector = this->desiredJointPos.prepare();
		
		for(int i = 0; i < this->numJoints; i++)
		{
			tempErrorVector.push_back(this->jointRef[i] - this->jointPos[i]);
			tempDesiredVector.push_back(this->jointRef[i]);
		}
		
		this->jointTrackingError.write();
		this->desiredJointPos.write();
		
		// Clear for the next loop
		tempErrorVector.clear();
		tempDesiredVector.clear();
		*/
		
		if(not send_joint_commands(this->jointRef)) std::cout << "[ERROR] [BIMANUAL CONTROL] Could not send joint commands for some reason.\n";
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void BimanualControl::threadRelease()
{
	send_joint_commands(this->jointPos);                                                        // Maintain current joint positions
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
		/* Can't use this because if the step size is too large then PositionDirect mode
		   on the motor controllers will cause insanely high joint torques
		
		lower = this->positionLimit[jointNum][0] - this->jointRef[jointNum];
		upper = this->positionLimit[jointNum][1] - this->jointRef[jointNum];
		*/
		
		/* This doesn't seem to work properly and I don't know why.
		lower = std::max(this->positionLimit[jointNum][0] - this->jointRef[jointNum],       // Distance to lower limit
		                -this->dt*this->velocityLimit[jointNum]);                           // Maximum speed
		                
		upper = std::min(this->positionLimit[jointNum][1] - this->jointRef[jointNum],       // Distance to upper limit
		                 this->dt*this->velocityLimit[jointNum]);                           // Maximum speed
		*/
		lower = std::max(this->positionLimit[jointNum][0] - this->jointRef[jointNum],       // Distance to lower limit
		                -0.1);                           // Maximum speed
		                
		upper = std::min(this->positionLimit[jointNum][1] - this->jointRef[jointNum],       // Distance to upper limit
		                 0.1);                           // Maximum speed
		if(lower >= upper)
		{
			std::cerr << "[ERROR] [BIMANUAL CONTROL] compute_joint_limits(): "
				  << "Lower limit " << lower << " is greater than upper limit " << upper << ". "
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
		
		this->objectPose = this->leftPose*Eigen::Translation3d(0,-graspWidth/2,0);          // Assume object is rigidly attached to left hand, half way to right hand
		
		this->object2Left = this->objectPose.inverse()*this->leftPose;                      // Desired left hand pose relative to control point of object  
		
		this->object2Right = this->objectPose.inverse()*this->rightPose;                    // Desired right hand pose relative to control point of object
		
		return true;
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
		return true;
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
	if(cartesian < 0 or cartesian >= 1.0 or redundant < 0 or redundant >= 1.0)
	{
		std::cerr << "[ERROR] [BIMANUAL CONTROL] set_control_gains(): "
		          << "Input arguments were " << cartesian << " and " << redundant << ", "
		          << "but must be between 0 and 1.\n";

		return false;
	}
	else
	{
		this->cartesianScalar = cartesian;
		this->redundantScalar = redundant;
		
		return true;
	}
}


