#include <CartesianTrajectory.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Isometry3d> &poses,
                                         const std::vector<double>            &times,
                                         const Eigen::Matrix<double,6,1>      &startVelocity)
                                         :
                                         numPoses(poses.size())
{
	// Check that the inputs are sound
	std::string errorMessage = "[ERROR] [CARTESIAN TRAJECTORY] Constructor: ";
	
	if(poses.size() != times.size())
	{
		errorMessage += "Pose vector had " + std::to_string(poses.size()) + " elements, and the "
		                "time vector had " + std::to_string(times.size()) + " elements.";
		
		throw std::invalid_argument(errorMessage);
	}
	else if(poses.size() < 2)
	{
		errorMessage += "A minimum of 2 poses is needed to create a trajectory.";
		
		throw std::invalid_argument(errorMessage);
	}
	else
	{
		std::vector<std::vector<double>> points; points.resize(6);                          // 6 dimension in 3D space
		for(int i = 0; i < 6; i++) points[i].resize(this->numPoses);
		
		// Extract all the positions and euler angles for each pose
		for(int i = 0; i < this->numPoses; i++)
		{
			// Extract the translation and put it in the points array
			Eigen::Vector3d translation = poses[i].translation();
			for(int j = 0; j < 3; j++) points[j][i] = translation[j];
			
			// Extract the angle & axis from the SO(3) matrix
			Eigen::Matrix3d R = poses[i].rotation();                                    // Get the rotation matrix
			double trace = R(0,0) + R(1,1) + R(2,2);                                    // Get the trace
			double angle = acos((trace - 1)/2);                                         // Compute the angle
			if(angle > M_PI) angle = 2*M_PI - angle;                                    // Ensure the range is [-3.14159, 3.14159]
			
			// Orientation component = angle*axis
			points[3][i] = angle*(R(2,1)-R(1,2));                                       // x component
			points[4][i] = angle*(R(0,2)-R(2,0));                                       // y component
			points[5][i] = angle*(R(1,0)-R(0,1));                                       // z component
			
			if(i > 0 and times[i] <= times[i-1])
			{
				errorMessage += "Times must be in ascending order. Waypoint "
				              + std::to_string(i-1) + " had a time of " + std::to_string(times[i-1])
				              + " and waypoint " + std::to_string(i) + " had a time of "
				              + std::to_string(times[i]) + ".";
				
				throw std::invalid_argument(errorMessage);
			}
		}
		 
		// Now insert them in to the iDynTree::CubicSpline object
		this->spline.resize(6);
		for(int i = 0; i < 6; i++)
		{
			if(not this->spline[i].setData(iDynTree::VectorDynSize(times), iDynTree::VectorDynSize(points[i])))
			{
				errorMessage += "Unable to set the spline data.";
				throw std::runtime_error(errorMessage);
			}
			else this->spline[i].setInitialConditions(startVelocity[i],0.0);
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired pose for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d CartesianTrajectory::get_pose(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no controls here if this function is called before the trajectory
	// is fully constructed...
	
	double pos[3];                                                                              // Position vector
	double rot[3];                                                                              // Angle*axis vector
	
	for(int i = 0; i < 3; i++)
	{
		pos[i] = this->spline[ i ].evaluatePoint(time);
		rot[i] = this->spline[i+3].evaluatePoint(time);
	}
	
	double angle = sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);                         // Norm of vector
	
	Eigen::Vector3d axis;
	
	if(angle == 0) axis = Eigen::Vector3d::UnitX();                                             // Axis is trivial
	else           axis = Eigen::Vector3d(rot[0]/angle,rot[1]/angle,rot[2]/angle);
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::AngleAxisd(angle,axis);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(Eigen::Isometry3d         &pose,
                                    Eigen::Matrix<double,6,1> &vel,
                                    Eigen::Matrix<double,6,1> &acc,
                                    const double              &time)
{
	double pos[3];                                                                              // Position vector
	double rot[3];                                                                              // Angle*axis

	for(int i = 0; i < 3; i++)
	{
		pos[i] = this->spline[ i ].evaluatePoint(time, vel[i]  , acc[i]);
		rot[i] = this->spline[i+3].evaluatePoint(time, vel[i+3], acc[i+3]);
	}

	double angle = sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);                         // Norm of vector
	
	Eigen::Vector3d axis;
	
	if(angle == 0) axis = Eigen::Vector3d::UnitX();                                             // Axis is trivial
	else           axis = Eigen::Vector3d(rot[0]/angle,rot[1]/angle,rot[2]/angle);
	
	pose = Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::AngleAxisd(angle,axis);

	return true;
}
