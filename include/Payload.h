    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //        A class that defines the dynamic properties of an object being carried by a robot       //   
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d
#include <iostream>                                                                                 // std::cout, std::cerr

class Payload
{
	public:
		Payload() : Payload(Eigen::Isometry3d(Eigen::Translation3d(0,0,0)*Eigen::Quaterniond(1,0,0,0))) {}
		
		Payload(const Eigen::Isometry3d &localPose) : _localPose(localPose) {}        
		
		Eigen::Isometry3d pose() const { return this->_globalPose; }      
		
		Eigen::Matrix<double,6,1> twist() const { return this->_twist; }
		
		void update_state(const Eigen::Isometry3d &globalToLocal,
		                  const Eigen::Matrix<double,6,1> &contactTwist);
		                  
		
	private:
		
		Eigen::Isometry3d _localPose;                                                       // Transform from contact point to centre of mass
		                                                    
		Eigen::Isometry3d _globalPose;                                                      // Pose in the global frame                                      
		
		Eigen::Matrix<double,6,1> _twist;                                                   // Linear & angular velocity
	
};                                                                                                  // Semicolon needed after class declaration

#endif
