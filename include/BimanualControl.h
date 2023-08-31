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
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops

class BimanualControl : public QPSolver<double>,
		 public yarp::os::PeriodicThread
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
		Eigen::Isometry3d object_pose() const { return this->global2Object; }

	private:
	
		unsigned int numJoints;
		
		bool isFinished = true;                                                             ///< For regulating control actions	
		
		double startTime, endTime;                                                          ///< For regulating the control loop
			
		std::vector<double> jointPos, jointVel, jointRef;
				
		Eigen::MatrixXd Jleft;                                                              ///< The left hand Jacobian matrix
		Eigen::MatrixXd Jright;                                                             ///< The right hand Jacobian matrix
		Eigen::MatrixXd J;                                                                  ///< The left and right hand Jacobian matrices stacked together                                                          
		
		Eigen::MatrixXd M;                                                                  ///< Inertia matrix, used to weight the control
		
		iDynTree::Transform basePose;                                                       ///< Needed by the iKinDynComputations class
		
		iDynTree::KinDynComputations computer;                                              ///< Does all the kinematics & dynamics
		
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 ///< Use for trajectory tracking of the joints
		
		double _limit = 0.001;                                                              ///< Minimum value for the manipulability
		
		CartesianTrajectory leftTrajectory;                                                 ///< Left hand trajectory generator
		
		CartesianTrajectory rightTrajectory;                                                ///< Right hand trajectory generator
		
		Eigen::Isometry3d leftPose;                                                         ///< The pose of the left hand
		
		Eigen::Isometry3d rightPose;                                                        ///< The pose of the right hand
		
		Eigen::Isometry3d global2Object;                                                    ///< (Desired) pose of the object in the world frame
		
		Eigen::Isometry3d leftHand2Object;
		
		bool isGrasping = false;                                                            ///< Used to check which control method to use
		
		CartesianTrajectory payloadTrajectory;                                              ///< Trajectory generator for a grasped object
		
		MotorControl motorController;                                                       ///< Communicates with the joint motors on the robot
		
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
		bool threadInit() { return true; }
		
		/**
		 * Executed after a control thread is stopped. Overridden from the PeriodicThread class.
		 */
		void threadRelease() {}
		
		/**
		 * Executes the main control loop. Overridden from the PeriodicThread class.
		 */
                void run() {}

};                                                                                                  // Semicolon needed after class declaration

#endif

