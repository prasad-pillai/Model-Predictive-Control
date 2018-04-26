#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// setting reference values
// this is what we want to achieve
#define REFERENCE_VELOCITY 40.0
#define REFERENCE_EPSI 0.0
#define REFERENCE_CTE 0.0

//Weights for the cost function
// This is manually tuned based on performance
//#define W_CTE 1.0
//#define W_EPSI 10.0
//#define W_V 1.0
//#define W_DELTA 500.0
//#define W_ACC 1.0
//#define W_DELTA_DIFF 25.0
//#define W_ACC_DIFF 1.0

#define W_CTE 1500.0
#define W_EPSI 500.0
#define W_V 1.0
#define W_DELTA 50.0
#define W_ACC 25.0
#define W_DELTA_DIFF 300.0
#define W_ACC_DIFF 125.0

// constrains for the variables
#define STEERING_BOUNDS 0.436332f //25 degrees in radians
#define ACC_BOUNDS 1.0
#define BOUNDS 1.0e19

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> res_x;
  vector<double> res_y;
};

#endif /* MPC_H */
