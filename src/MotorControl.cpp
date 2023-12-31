#include <MotorControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
MotorControl::MotorControl(const std::vector<std::string> &jointList,
                           const std::vector<std::string> &portList)
                           :
                           numJoints(jointList.size())                                              // Number of joints equal to size of list
{
	// Resize std::vector objects based on number of joints in the model
	this->positionLimit.resize(this->numJoints);
	this->velocityLimit.resize(this->numJoints);

	////////////////////////// I copied this code from elsewhere ///////////////////////////////

	// Open up device drivers
	yarp::os::Property options;
	options.put("device", "remotecontrolboardremapper");
	options.addGroup("axesNames");

	yarp::os::Bottle & bottle = options.findGroup("axesNames").addList();
	for(int i = 0; i < jointList.size(); i++) bottle.addString(jointList[i].c_str());           // Add the list of all the joint names

	yarp::os::Bottle remoteControlBoards;
	yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
	for(int i = 0; i < portList.size(); i++) remoteControlBoardsList.addString(portList[i]);    // Add the remote control board port names

	options.put("remoteControlBoards", remoteControlBoards.get(0));
	options.put("localPortPrefix", "/local");

	yarp::os::Property &remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
			    remoteControlBoardsOpts.put("writeStrict", "on");

	////////////////////////////////////////////////////////////////////////////////////////////
	
	std::string errorMessage = "[ERROR] [MOTOR CONTROL] Constructor: ";

	if(not this->driver.open(options)) throw std::runtime_error(errorMessage + "Could not open the device driver.");
	else
	{
		     if(not this->driver.view(this->controller))  throw std::runtime_error(errorMessage + "Unable to configure the position control for the joint motors.");
		else if(not this->driver.view(this->mode))        throw std::runtime_error(errorMessage + "Unable to configure the control mode.");
		else if(not this->driver.view(this->limits))      throw std::runtime_error(errorMessage + "Unable to obtain the joint limits.");
		else
		{
			// Opened the motor controllers, so get the joint limits
			for(int i = 0; i < this->numJoints; i++)
			{
				double notUsed;
				this->limits->getLimits(i, &this->positionLimit[i][0], &this->positionLimit[i][1]);
				this->limits->getVelLimits(i, &notUsed, &this->velocityLimit[i]);   // Assume vMin = -vMax
				
				// Convert from degrees to radians
				this->positionLimit[i][0] *= M_PI/180.0;
				this->positionLimit[i][1] *= M_PI/180.0;
				this->velocityLimit[i]    *= M_PI/180.0;
				
				if(not this->mode->setControlMode(i,VOCAB_CM_POSITION_DIRECT))
				{
					throw std::runtime_error(errorMessage + "Unable to set the control mode for joint " + std::to_string(i) + ".");
				}
			}
			
			// Finally, configure the encoders
			if(not this->driver.view(this->encoders)) throw std::runtime_error(errorMessage + "Unable to configure the encoders.");
			else
			{
				double temp[this->numJoints];                                       // Temporary placeholder for encoder values			

				// Make 5 attempts to read the encoders
				for(int i = 0; i < 5; i++)
				{
					if(not this->encoders->getEncoders(temp) and i == 4)
					{
						throw std::runtime_error(errorMessage + "Could not obtain encoder values in 5 attempts.");
					}
					
					yarp::os::Time::delay(0.5);                                 // Wait a little bit before trying again
				}
				
				std::cout << "[INFO] [MOTOR CONTROL] Successfully configured the joint motors.\n";
			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Read the joint positions and velocities from the encoders                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool MotorControl::read_encoders(std::vector<double> &pos, std::vector<double> &vel)
{
	if(pos.size() != this->numJoints or vel.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [MOTOR CONTROL] read_encoders(): "
		          << "There are " << this->numJoints << " joints, but "
		          << "the position argument had " << pos.size() << " elements and "
		          << "the velocity argument had " << vel.size() << " elements.\n";

		return false;
	}
	else
	{
		bool success = true;
		
		for(int i = 0; i < this->numJoints; i++)
		{
			success &= this->encoders->getEncoder     (i, &pos[i]);                     // Read joint positions
			success &= this->encoders->getEncoderSpeed(i, &vel[i]);                     // Read joint velocities
			
			// Convert to radians
			pos[i] *= M_PI/180.0;
			vel[i] *= M_PI/180.0;
		}
		
		if(success) return true;
		else
		{
			std::cerr << "[ERROR] [MOTOR CONTROL] read_encoders(): Could not obtain new encoder values.\n";
			
			return false;
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Send commands to the joint motors                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool MotorControl::send_joint_commands(const std::vector<double> &commands)
{
	if(commands.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [MOTOR CONTROL] send_joint_command(): "
		          << "This robot has " << this->numJoints << " active joints but the input "
		          << "argument had " << commands.size() << " elements.\n";
		          
	        return false;
	}
	else
	{
		for(int i = 0; i < this->numJoints; i++)
		{	
			if(not this->controller->setPosition(i,commands[i]*180/M_PI))               // NOTE: Need to convert to degrees
			{
				std::cerr << "[ERROR] [MOTOR CONTROL] send_joint_commands(): "
				          << "Could not send a command for joint " << i << ".\n";
				
				return false;
			}
		}
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Close the device interfaces on the robot                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void MotorControl::close()
{
	for(int i = 0; i < this->numJoints; i++) this->mode->setControlMode(i, VOCAB_CM_POSITION);  // Set in position mode to lock the joint
	
	this->driver.close();                                                                       // Close the device drivers
}

