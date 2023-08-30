    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                       Base class for bimanual control of the iCub                              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUBBASE_H_
#define ICUBBASE_H_

#include <CartesianTrajectory.h>
#include <Eigen/Dense>                                                                              // Tensors and matrix decomposition
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen tensors
#include <iDynTree/Core/CubicSpline.h>                                                              // Uses for joint trajectories
#include <iDynTree/Model/FreeFloatingState.h>                                                       // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/KinDynComputations.h>                                                            // Class for invers dynamics calculations
#include <iDynTree/Model/Model.h>                                                                   // Class that holds info on kinematic tree structure
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Class for communicating with joint motors
#include <Payload.h>
#include <QPSolver.h>                                                                               // For control optimisation
#include <yarp/os/PeriodicThread.h>                                                                 // Class for timing control loops

class iCubBase : public QPSolver,
		 public JointInterface,
		 public yarp::os::PeriodicThread
{
	public:
		iCubBase(const std::string              &pathToURDF,
			 const std::vector<std::string> &jointList,
		         const std::vector<std::string> &portList,
		         const std::string              &robotModel);

		bool update_state();                                                                // Read encoders, compute kinematics & dynamics
		
		bool is_finished() const { return this->isFinished; }                               // Returns true when trajectory reaches its end
		
		void halt();                                                                        // Stops the robot immediately
		
		// Joint control functions
		bool move_to_position(const Eigen::VectorXd &position,
		                      const double &time);                                          // Move the joints to a given position
		                      
		bool move_to_positions(const std::vector<Eigen::VectorXd> &positions,
		                       const std::vector<double> &times);                           // Move the joints through several positions
		                       
		bool set_joint_gains(const double &proportional, const double &derivative);         // Set the gains for joint control
		
		// Cartesian control functions
		bool is_grasping() const { return this->isGrasping; }                               // Check if grasp constraint is active
		
		bool move_to_pose(const Eigen::Isometry3d &desiredLeft,
		                  const Eigen::Isometry3d &desiredRight,
		                  const double &time);                                              // Move each hand to a pose
		                  
		bool move_to_poses(const std::vector<Eigen::Isometry3d> &left,
		                   const std::vector<Eigen::Isometry3d> &right,
		                   const std::vector<double> &times);                               // Move hands through several poses
		                   
		bool translate(const Eigen::Vector3d &left,
		               const Eigen::Vector3d &right,
		               const double &time);                                                 // Translate the hands by a specific amount
		               
		Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
		                                     const Eigen::Isometry3d &actual);
		                                     
		Eigen::Isometry3d hand_pose(const std::string &which);                              // Get the pose of a specified hand
		                        
		Eigen::Isometry3d object_pose() const { return this->payload.pose();}         
		                                     
		bool set_cartesian_gains(const double &proportional, const double &derivative);     // Set the gains for the controller
		
		bool set_desired_joint_position(const Eigen::VectorXd &position);                   // For redundancy resolution                   
		         
		bool set_singularity_avoidance_params(const double &_maxDamping,
		                                      const double &_threshold);
		                                      
		bool set_redundant_task(const std::string &task, const double &scalar);		         
		               
		// Grasp control functions
		bool grasp_object();                                                                // As it says on the label
		
		bool release_object();                                                              // Deactivates grasp constraints
		
		bool move_object(const Eigen::Isometry3d &pose, const double &time);                // Move the grasped object to a single pose
		              
		bool move_object(const std::vector<Eigen::Isometry3d> &poses, const std::vector<double> &times); // Move object through multiple waypoints	       

	protected:
	
		std::string _robotModel;                                                            // iCub2, iCub3, or ergoCub
		
		enum ControlSpace {joint, cartesian} controlSpace;
		
		enum RedundantTask {setPoint, singularityAvoidance} redundantTask = singularityAvoidance;
		
		bool isFinished = true;                                                             // For regulating control actions	
		
		double startTime, endTime;                                                          // For regulating the control loop
		
		double dt = 0.01;
			
		// Kinematics & dynamics
		
		Eigen::VectorXd q, qdot, coriolisAndGravity;                                        // Joint positions and velocities
		
		Eigen::MatrixXd Jleft, Jright, J, M, invM;                                          // Jacobian, inertia, and inverse of inertia
		
		iDynTree::Transform basePose;                                                       // Pose of the base
		
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		
		iDynTree::FreeFloatingGeneralizedTorques generalForces;                             // Class for storing information
		
		// Joint control
		
		double kp = 1.0;                                                                    // Default proportional gain
		
		double kd = 2*sqrt(this->kp);                                                       // Theoretically optimal damping
		
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // As it says
		
		// Cartesian control
		
		double maxDamping = 0.01;                                                           // For singularity avoidance
		
		double threshold  = 0.001;                                                          // For activating singularity avoidance
	
		double manipulability;                                                              // Proximity to a singularity
			
		Eigen::Matrix<double,6,6> gainTemplate = (Eigen::MatrixXd(6,6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
		                                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.5).finished();                    
		Eigen::Matrix<double,6,6> K = 1*this->gainTemplate;
		
		Eigen::Matrix<double,6,6> D = 2*this->gainTemplate;                                 // Theoretically optimal: kd >= 2*sqrt(kp)
		
		double kr = 0.1;                                                                    // Gain for the redundant task
		
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Trajectory generators for the hands
		
		Eigen::Isometry3d leftPose, rightPose;                                              // Left and right hand pose
		
		Eigen::VectorXd desiredPosition;                                                    // For redundancy resolution
		
		Eigen::VectorXd redundant_task();                                                   // Compute the desired redundant task
		
		// Grasp control
		
		bool isGrasping = false;
		
		Eigen::Matrix<double,6,12> G, Gdot;                                                 // Grasp matrix
		
		Eigen::Matrix<double,6,12> C, Cdot;                                                 // Constraint matrix
		
		Payload payload;                                                                    // Class for representing object being held
		
		CartesianTrajectory payloadTrajectory;                                              // Trajectory generator for a grasped object
		
		Eigen::Isometry3d desiredLeft2Right;                                                // Desired pose of left hand to right hand when grasping
	
	private:
		
		// Functions
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);                  // Convert Eigen::Isometry to iDynTree::Transform
		
		Eigen::Isometry3d iDynTree_to_Eigen(const iDynTree::Transform &T);                  // Convert iDynTree:Transform to Eigen::Isometry
		
		Eigen::Vector3d angle_axis(const Eigen::Matrix3d &R);                               // Convert rotation matrix to its angle and axis
		
		Eigen::MatrixXd partial_derivative(const Eigen::MatrixXd &J, const unsigned int &jointNum); // Get the partial derivative of a Jacobian
		
		// NOTE: THESE FUNCTIONS MUST BE DECLARED IN ANY CHILD CLASS OF THIS ONE
		virtual bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum) = 0;
		
		virtual Eigen::VectorXd track_joint_trajectory(const double &time) = 0;
		
		virtual Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time) = 0;
	
		// NOTE: THESE FUNCTIONS ARE FROM THE PERIODICTHREAD CLASS AND NEED TO BE DEFINED
		// IN THE CHILD CLASSES OF THIS ONE
//		bool threadInit() { return true; }
//		void threadRelease() {}
//              void run() {}                                                                       // Defined in iCub2.h

};                                                                                                  // Semicolon needed after class declaration

#endif
