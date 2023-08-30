    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for interfacing with the joint motors on the iCub                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <Eigen/Core>                                                                               // Eigen::VectorXd
#include <iostream>                                                                                 // std::cerr, std::cout
#include <math.h>                                                                                   // M_PI
#include <string>                                                                                   // std::string
#include <vector>                                                                                   // std::vector
#include <yarp/dev/ControlBoardInterfaces.h>                                                        // I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>                                                                    // ... or this...
#include <yarp/os/Property.h>                                                                       // ... or this.

class JointInterface
{
	public:		
		JointInterface(const std::vector<std::string> &jointList,                           // Constructor
		               const std::vector<std::string> &portList);
		
		bool read_encoders(Eigen::VectorXd &pos, Eigen::VectorXd &vel);                     // Read the positions and velocities
		
		bool send_joint_commands(const Eigen::VectorXd &commands);
		
		void close();                                                                       // Close the device drivers & stop the robot
		
        protected:
     
		unsigned int numJoints;                                                             // Number of joints being controlled
		
		std::vector<std::array<double,2>> positionLimit;                                    // Upper and lower bounds on joint position
		
		std::vector<double>               velocityLimit;                                    // Absolute joint velocity
		
		yarp::dev::IControlMode*     mode;                                                  // Sets the control mode of the motor
	private:
		
	   	// These interface with the hardware on the robot itself
		yarp::dev::IControlLimits*   limits;                                                // Joint limits?
		yarp::dev::IEncoders*        encoders;                                              // Joint position values (in degrees)
		yarp::dev::ITorqueControl*   controller;                                            // As it says
		yarp::dev::PolyDriver        driver;                                                // Device driver
	
};                                                                                                  // Semicolon needed after class declaration

#endif
