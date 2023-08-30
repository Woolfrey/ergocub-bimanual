  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Cartesian impedance control for the ergoCub robot                         //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FORCE_CONTROL_H_
#define FORCE_CONTROL_H_

#include <iCubBase.h>

class ForceControl : public iCubBase
{
	public:
		ForceControl(const std::string              &pathToURDF,
			     const std::vector<std::string> &jointList,
			     const std::vector<std::string> &portList,
			     const std::string              &robotModel)
		:
	        iCubBase(pathToURDF, jointList, portList, robotModel) {}
	        
		//////////////////////// Inherited from iCubBase class ////////////////////////////
		bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		///////////////////////////////////////////////////////////////////////////////////
		
	protected:
	
		bool threadInit();
		void run();
		void threadRelease();
};                                                                                                  // Semicolon needed after class declaration

#endif

