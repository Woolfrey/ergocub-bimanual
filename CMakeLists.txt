cmake_minimum_required(VERSION 2.8.9)					                            # Minimum cmake requirement

project(icub-bimanual)							                            # ${PROJECT_NAME}

# cmake -DCMAKE_BUILD_TYPE=Release ..

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)                                                               # Release = Optimised
endif()

#################################### Download QPSolver #############################################

if(EXISTS "${CMAKE_SOURCE_DIR}/include/QPSolver.h")
	message([STATUS] " Found the QPSolver class in ${CMAKE_SOURCE_DIR}/include/")
else()
	message([WARNING] " QPSolver class not found in ${CMAKE_SOURCE_DIR}/include/")
	
	message([STATUS] " Downloading from https://github.com/Woolfrey/SimpleQPSolver ...")
	
	file(DOWNLOAD
	    https://raw.githubusercontent.com/Woolfrey/SimpleQPSolver/master/include/QPSolver.h
	    ${CMAKE_SOURCE_DIR}/include/QPSolver.h)
	    
	if(EXISTS "${CMAKE_SOURCE_DIR}/include/QPSolver.h")
		message([STATUS] " ... Done!")
	else()
		message([FATAL ERROR] " Could not download QPSolver.")
	endif()
endif()

################################## Necessary packages to compile ###################################
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)                                                                     # Links below to iDynTree
find_package(YARP 3.3.0 REQUIRED)                                                                   # Links below to ${YARP_LIBRARIES}

#################################### Other files to be built #######################################
add_subdirectory(gazebo)                                                                            # Location of other CMakeLists
add_subdirectory(interface)                                                                         # Location & definition of thrift interface
include_directories(include)                                                                        # Location of header files

#################################### Executables to be compiled ####################################
add_executable(command_server src/BimanualControl.cpp src/CartesianTrajectory.cpp src/ControlInterface.cpp src/ControlServer.cpp src/HumanState.cpp src/MotorControl.cpp src/Quaternion.cpp src/Utilities.cpp src/Vector3.cpp)
target_link_libraries(command_server Eigen3::Eigen iDynTree::idyntree-high-level ${YARP_LIBRARIES})

add_executable(command_prompt src/CommandPrompt.cpp src/ControlInterface.cpp src/Utilities.cpp)
target_link_libraries(command_prompt Eigen3::Eigen ${YARP_LIBRARIES})
