#ifndef QPSOLVER_H_
#define QPSOLVER_H_

#include <Eigen/Dense>                                                                              // Eigen::MatrixXd and matrix decomposition
#include <iostream>                                                                                 // std::cout, std::cerr
#include <math.h>
#include <vector>                                                                                   // std::vector

class QPSolver
{
	public:
		QPSolver() {}
		
		// These functions can be called without creating a QPSolver object:
		static Eigen::VectorXd solve(const Eigen::MatrixXd &H,                              // Solve a generic QP problem
		                             const Eigen::VectorXd &f);
		                             
		static Eigen::VectorXd least_squares(const Eigen::VectorXd &y,
		                                     const Eigen::MatrixXd &A,
		                                     const Eigen::MatrixXd &W);                     // Solve a standard least squares problem
		                                     
		static Eigen::VectorXd redundant_least_squares(const Eigen::VectorXd &xd,
		                                               const Eigen::MatrixXd &W,
		                                               const Eigen::VectorXd &y,
		                                               const Eigen::MatrixXd &A);           // Solve a redundant least squares problem
		                                      
		// These functions require an object to be created since they use the
		// interior point solver:
		Eigen::VectorXd solve(const Eigen::MatrixXd &H,                                     // Solve QP problem with inequality constraints
		                      const Eigen::VectorXd &f,
		                      const Eigen::MatrixXd &B,
		                      const Eigen::VectorXd &z,
		                      const Eigen::VectorXd &x0);
		                                                   
		Eigen::VectorXd least_squares(const Eigen::VectorXd &y,                             // Solve a constrained least squares problem
		                              const Eigen::MatrixXd &A,
		                              const Eigen::MatrixXd &W,
		                              const Eigen::VectorXd &xMin,
		                              const Eigen::VectorXd &xMax,
		                              const Eigen::VectorXd &x0);
		                              
		Eigen::VectorXd redundant_least_squares(const Eigen::VectorXd &xd,                   // Solve a constrained least squares problem
		                                        const Eigen::MatrixXd &W,
		                                        const Eigen::VectorXd &y,
		                                        const Eigen::MatrixXd &A,
		                                        const Eigen::VectorXd &xMin,
		                                        const Eigen::VectorXd &xMax,
		                                        const Eigen::VectorXd &x0);
		                              
		Eigen::VectorXd last_solution() const { return this->lastSolution; }                // As it says on the label
		
		void clear_last_solution() { this->lastSolutionExists = false; }                    // Clear the last solution
		
		bool last_solution_exists() const { return this->lastSolutionExists; }
		                         
	private:
		// These are variables used by the interior point method:
		float alpha0    = 1.0;                                                              // Scalar for Newton step
		float alphaMod  = 0.5;                                                              // Modify step size when constraint violated
		float beta0     = 0.01;                                                             // Rate of decreasing barrier function
		float tol       = 1e-2;                                                             // Tolerance on step size
		float u0        = 100;                                                              // Scalar on barrier function
		int   steps     = 20;                                                               // No. of steps to run interior point method
		
		bool lastSolutionExists = false;
		
		Eigen::VectorXd lastSolution;
		                         
};                                                                                                  // Semicolon needed after class declaration

#endif
