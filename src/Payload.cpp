#include <Payload.h>

void Payload::update_state(const Eigen::Isometry3d &globalToLocal,
                           const Eigen::Matrix<double,6,1> &contactTwist)
{
	this->_globalPose = globalToLocal*this->_localPose;                                         // Compute pose of object in global frame
	                    
	Eigen::Vector3d r = this->_globalPose.translation() - globalToLocal.translation();
	Eigen::Vector3d w = contactTwist.tail(3);
	
	this->_twist.head(3) = contactTwist.head(3) + w.cross(r);
	this->_twist.tail(3) = w;
}
