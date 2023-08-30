    ////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                        //
  //           Set up a client to allow manual prompts with the command server              //
 //                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////

#include <CommandInterface.h>                                                                       // Definition of thrift interface
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Utilities.h>                                                                              // For loading parameters from config files
#include <yarp/os/Network.h>                                                                        // Load configuration files
#include <yarp/os/Property.h>                                                                       // For loading paramters from config files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

std::string errorMessage = "[ERROR] [COMMAND PROMPT] ";

std::map<std::string,int> commandList;                                                             // 0 = Joint command, 1 = Cartesian command

int main(int argc, char* argv[])
{
	// Minimum is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 3)
	{
		std::cerr << errorMessage << "Path to configuration file required. "
		          << "Usage: ./command_prompt /serverPortName /path/to/config.ini\n";
		          
		return 1;
	}
	
	std::string serverPortName = argv[1];
	
	// Load the list of predefined joint configuration names
	yarp::os::Property parameter; parameter.fromConfigFile(argv[2]);                            // Load the properties from the config file
	yarp::os::Bottle* bottle = parameter.findGroup("JOINT_SPACE_ACTIONS").find("names").asList();
	std::vector<std::string> names = string_from_bottle(bottle);                                // Get the list of the configuration names
	for(int i = 0; i < names.size(); i++) commandList.emplace(names[i],0);                      // Put them in to the map
	
	// Load the list of Cartesian actions
	bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("names").asList();
	names = string_from_bottle(bottle);
	for(int i = 0; i < names.size(); i++) commandList.emplace(names[i],1);
	
	// Load the list of grasp actions
	bottle->clear(); bottle = parameter.findGroup("GRASP_ACTIONS").find("names").asList();
	names = string_from_bottle(bottle);
	for(int i = 0; i < names.size(); i++) commandList.emplace(names[i],2);
	
	yarp::os::Network yarp; 
	
	// Connect this client port with the command server       
	yarp::os::Port clientPort; 
	
	if(not clientPort.open("/commandClient"))
	{
		std::cerr << errorMessage << "Could not open port under the name '/commandClient' \n";
		return 1;
	}
	
	if(not yarp.connect("/commandClient", serverPortName))
	{
		std::cerr << errorMessage << "Could not connect '/commandClient' with '/commandServer'.\n";
		return 1;
	}
	
	CommandInterface client;
	client.yarp().attachAsClient(clientPort);                                                   // As it says on the label
	
	// Set up an input port here to pass on manual commands
	yarp::os::RpcServer promptPort;                                                             // Create a port for sending / receiving info
	promptPort.open("/commandPrompt");                                                          // Open the port with the name '/command'
	yarp::os::Bottle input;                                                                     // Store information from the user input
	yarp::os::Bottle output;                                                                    // Store information to send to the user
	
	bool active = true;
	
	while(active)
	{
		output.clear();                                                                     // Clear any previous output
		promptPort.read(input,true);                                                        // Read the port for manual commands
		
		std::string command = input.toString();                                             // Convert to string
		
		if(command == "close")
		{
			output.addString("Arrivederci");
			client.shut_down();                                                         // Shut down the server
			active = false;                                                             // Exit and shut down this
		}
		else if(command == "stop")
		{
			output.addString("Fermata");
			
			client.stop();
		}
		else if(command == "grasp")
		{
			output.addString("Mio");
			client.grasp();
		}
		else if(command == "release")
		{
			output.addString("Capito");
			client.release_object();
		}
		else
		{
			auto blah = commandList.find(command);
			
			if(blah == commandList.end())
			{
				output.addString("Cosa");
			}
			else
			{
				if(blah->second == 0)                                               // It's a joint command
				{
					if(client.perform_joint_space_action(command)) output.addString("Capito");
					else output.addString("Problema");
				}
				else if(blah->second == 1)                                          // It's a Cartesian command
				{
					if(client.perform_cartesian_action(command)) output.addString("Capito");
					else output.addString("Problema");
				}
				else if(blah->second == 2)
				{
					if(client.perform_grasp_action(command)) output.addString("Capito");
					else output.addString("Problema");
				}
				else output.addString("Problema");
			}
		}
			
		promptPort.reply(output);
	}
	
	return 0;
}
