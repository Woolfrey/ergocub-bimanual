/**
 * @file   BimanualControl.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Header containing class definitions for two-arm control of the ergoCub
 */

#ifndef BIMANUALCONTROL_H_
#define BIMANUALCONTROL_H_

#include <CartesianTrajectory.h>                                                                    // Custom class
#include <Eigen/Dense>                                                                              // Tensors and matrix decomposition
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen tensors
#include <iDynTree/Core/CubicSpline.h>                                                              // Uses for joint trajectories
#include <iDynTree/Model/FreeFloatingState.h>                                                       // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations
#include <iDynTree/Model/Model.h>                                                                   // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <MotorControl.h>                                                                           // Class for communicating with joint motors
#include <QPSolver.h>                                                                               // For control optimisation
#include <WalkingControllers/YarpUtilities/HumanState.h>                                            // Needed to integrate with AMI's walking controller
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops
#include <yarp/sig/Vector.h>                                                                        // For sending data over YARP ports

class BimanualControl : public QPSolver<double>,
                        public yarp::os::PeriodicThread,
                        public MotorControl
{
	public:
		/**
		 * Constructor.
		 * @param pathToURDF Location of the URDF file that describes the robot.
		 * @param jointList  The list of joints to control, in order
		 * @param portList   The names of the YARP ports for the joints
		 */
		BimanualControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointList,
		                const std::vector<std::string> &portList);

		/**
		 * Reads the joint position and updates the forward kinematics.
		 * @return True if successful.
		 */
		bool update_state();
		
		/** Checks to see if the robot has finished an action.
		 * @return Returns true if finished.
		 */
		bool is_finished() const { return this->isFinished; }
		
		/**
		 * Immediately stops the robot from moving.
		 */
		void halt();
		
		/**
		 * Moves the joints to a desired position.
		 * @param position The desired joint positions.
		 * @param time The time in which to execute the movement.
		 * @return Returns true if there were no problems.
		 */
		bool move_to_position(const Eigen::VectorXd &position,
		                      const double &time);
		                      
		/**
		 * Moves the joints through several positions using spline interpolation.
		 * @param positions An array of positions to move between.
		 * @param times The time at which to reach each position.
		 * @return Returns true if there were no problems.
		 */
		bool move_to_positions(const std::vector<Eigen::VectorXd> &positions,
		                       const std::vector<double> &times);
		
		/**
		  * Check to see if the robot is grasping an object.
		  */
		bool is_grasping() const { return this->isGrasping; }
		
		/**
		 * Move each hand to a desired pose.
		 * @param desiredLeft The desired pose for the left hand
		 * @param desiredRight The desired pose for the right hand
		 * @param time The time in which to execute the motion.
		 * @return Returns true if there were no problems.
		 */
		bool move_to_pose(const Eigen::Isometry3d &desiredLeft,
		                  const Eigen::Isometry3d &desiredRight,
		                  const double &time);
		
		/**
		 * Move each hand through several poses.
		 * @param left An array of poses for the left hand,
		 * @param right An array of poses for the right hand,
		 * @param times The time to reach the poses.
		 * @return Returns true if there were no problems.
		 */
		bool move_to_poses(const std::vector<Eigen::Isometry3d> &left,
		                   const std::vector<Eigen::Isometry3d> &right,
		                   const std::vector<double> &times);
		                   
		/**
		 * Translate the hands by specified distance.
		 * @param left A 3D vector for the left hand translation
		 * @param right A 3D vector for the right hand translation
		 * @time time The time in which to execute the motion
		 * @return Returns true if there were no problems
		 */
		bool translate(const Eigen::Vector3d &left,
		               const Eigen::Vector3d &right,
		               const double &time);
		               
		/**
		 * Get the error between two poses
		 * @param desired The desired pose
		 * @param actual The actual pose
		 * @return A 6x1 vector containing the translation and orientation error.
		 */
		Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
		                                     const Eigen::Isometry3d &actual);

		/**
		 * Set the parameters for the singularity avoidance.
		 * @param scalar A small value will make the control more conservative.
		 * @param limit The minimum possible value for the manipulability (proximity to a singularity)
		 * @return Returns true if there were no problems.
		 */
		bool set_singularity_avoidance_params(const double &scalar,
		                                      const double &limit);
		                 
		/**
		 * A secondary objective for configuring the arms in Cartesian control mode. It is reset every loop.
		 * @param task A discrete joint step to be executed
		 * @param scalar A scalar on the step size
		 * @return Returns true if there were no problems.
		 */
		bool set_redundant_task(const std::string &task, const double &scalar);		         
		               
		/**
		 * Activates grasping so the hands move together when holding an object.
		 */
		bool activate_grasp();
		
		/**
		 * Deactives grasping so the hands can move independently.
		 */
		bool release_grasp();
		
		/**
		 * Move a grasped object to a desired pose.
		 * @param pose The desired pose for the object
		 * @param time The time in which to execute the motion
		 * @return Returns true if there were no problems.
		 */
		bool move_object(const Eigen::Isometry3d &pose, const double &time);
		
		/**
		 * Move a grasped object through several poses.
		 * @param poses An array of desired poses.
		 * @param times The time to reach each pose.
		 * @return Returns true if there were no problems.
		 */
		bool move_object(const std::vector<Eigen::Isometry3d> &poses, const std::vector<double> &times);
		
		/**
		 * Return the current pose of the specified hand.
		 * @param which The name of the hand, should be either "left" or "right"
		 * @return Hand pose represented by an Eigen::Isometry3d object
		 */
		Eigen::Isometry3d hand_pose(const std::string &which);
		
		/**
		 * Return the pose of an object being grasped
		 */
		Eigen::Isometry3d object_pose() const { return this->objectPose; }
		
		/**
		 * Set the gains for different control tasks.
		 * @param cartesian The gain when controlling the hands
		 * @param redundant The gain when reconfiguring the arms in Cartesian mode
		 * @return Returns true if there were no problems
		 */
		 bool set_control_gains(const double &cartesian, const double &redundant);
		 
		 /**
		  * Set the motor control on or off.
		  * @param active If true, then joint commands will be sent to the motors. Set false if using with walking controller, for example.
		  */
		 void motor_control(const bool &active);

	private:
		bool isGrasping = false;                                                            ///< Used to check which control method to use
		
		bool isFinished = true;                                                             ///< For regulating control actions
		
		bool motorControlActive = false;                                                    ///< Will send commands to motor control
		
		double startTime, endTime;                                                          ///< For regulating the control loop
		
		double dt = 0.01;                                                                   ///< 1/frequency

		double barrierScalar = 10.0;                                                        ///< Used for avoiding singular regions
		
		double manipulability;                                                              ///< Proximity to a singular configuration
		
		double manipulabilityLimit = 0.001;                                                 ///< Minimum value for the manipulability
		
		double cartesianScalar = 1.0;                                                       ///< Scalar on Cartesian feedback
		
		double redundantScalar = 1.0;                                                       ///< Scalar on redundant task
		
		std::vector<double> jointPos;                                                       ///< Actual joint state read from joint encoders
		
		std::vector<double> jointRef;                                                       ///< Reference positions used for all control
		
		std::vector<double> jointVel;                                                       ///< Joint velocities read from joint encoders
		
		std::vector<std::string> jointNames;                                                ///< Names of all the joints being controlled
		
		yarp::os::BufferedPort<yarp::sig::Vector> jointReferences;                          ///< Data on joint references
		
		yarp::os::BufferedPort<yarp::sig::Vector> jointTrackingError;                       ///< For sending performance data over YARP port
		
		yarp::os::BufferedPort<yarp::sig::Vector> constraintAdherence;                      ///< For measuring how well constraints are enforced
		
		yarp::os::BufferedPort<yarp::sig::Vector> objectTrackingError;                      ///< As it says
		
		yarp::os::BufferedPort<yarp::sig::Vector> manipulabilityData;                       ///< Minimum manipulability

		yarp::os::BufferedPort<yarp::os::Bottle> neckYawReferences;							//Port to read neck reference for gaze control

		yarp::os::BufferedPort<WalkingControllers::YarpUtilities::HumanState> walkingModuleInterface; ///< YARP port for interfacing with AMI's walking controller
				
		Eigen::MatrixXd Jleft;                                                              ///< The left hand Jacobian matrix
		
		Eigen::MatrixXd Jright;                                                             ///< The right hand Jacobian matrix
		
		Eigen::MatrixXd J;                                                                  ///< The left and right hand Jacobian matrices stacked together                                                          

		Eigen::Matrix<double,6,12> G;                                                       ///< Grasp matrix
		
		Eigen::Matrix<double,6,12> C;		                                            ///< Constraint matrix
		
		Eigen::MatrixXd M;                                                                  ///< Inertia matrix, used to weight the control
		
		Eigen::Matrix<double,6,6> gainTemplate = (Eigen::MatrixXd(6,6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
						                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
						                                  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
						                                  0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
						                                  0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
						                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.5).finished();  
						                                                                    
		Eigen::Matrix<double,6,6> K = 1.0*this->gainTemplate;
		
		Eigen::Vector<double,6> objectPoseError;                                            ///< For assessing performance
		
		iDynTree::Transform basePose;                                                       ///< Needed by the iKinDynComputations class
		
		iDynTree::KinDynComputations computer;                                              ///< Does all the kinematics & dynamics
		
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 ///< Use for trajectory tracking of the joints
		
		CartesianTrajectory leftTrajectory;                                                 ///< Left hand trajectory generator
		
		CartesianTrajectory rightTrajectory;                                                ///< Right hand trajectory generator
		
		CartesianTrajectory objectTrajectory;                                               ///< Trajectory generator for a grasped object
		
		Eigen::Isometry3d leftPose;                                                         ///< The pose of the left hand
		
		Eigen::Isometry3d rightPose;                                                        ///< The pose of the right hand
		
		Eigen::Isometry3d objectPose;                                                       ///< Pose of the object
		
		Eigen::Isometry3d leftHand2Object;                                                  ///< Pose of the object relative to left hand
		
		/**
		 * Specifies joint control mode, or Cartesian control mode.
		 */
		enum ControlSpace {joint, cartesian} controlSpace;
		
		/**
		 * Converts an Eigen::Isometry3d object to an iDynTree::Transform object
		 */
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);
		
		/**
		 * Converts an iDynTree::Transform object to an Eigen::Isometry3d object
		 */
		Eigen::Isometry3d iDynTree_to_Eigen(const iDynTree::Transform &T);
		
		/**
		 * Extracts the angle*axis vector from a rotation matrix.
		 */
		Eigen::Vector3d angle_axis(const Eigen::Matrix3d &R);
		
		/**
		 * Computes the partial derivative of a Jacobian matrix with respect to a joint; dJ/dq_i
		 * @param J The Jacobian matrix (6xn)
		 * @param jointNum The number the joint with which to take the derivative
		 * @return The partial derivative (6xn)
		 */
		Eigen::MatrixXd partial_derivative(const Eigen::MatrixXd &J, const unsigned int &jointNum);
		
		/**
		 * Compute the instantaneous control limits on a given joint.
		 * @param lower The lower bound value (to be computed)
		 * @param upper The upper bound value (to be computed)
		 * @param jointNum The joint number for which the limits are computed
		 * @return Returns true if there were no problems
		 */
		bool compute_control_limits(double &lower, double &upper, const unsigned int &jointNum);
			
		/**
		 * Initialise the control thread. Overriden from the PeriodicThread class.
		 */
		bool threadInit();
		
		/**
		 * Executed after a control thread is stopped. Overridden from the PeriodicThread class.
		 */
		void threadRelease();
		
		/**
		 * Executes the main control loop. Overridden from the PeriodicThread class.
		 */
                void run();

};                                                                                                  // Semicolon needed after class declaration

#endif

