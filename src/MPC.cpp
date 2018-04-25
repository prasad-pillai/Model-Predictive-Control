#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 15;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//state variable sizes
size_t start_x = 0;
size_t start_y = start_x + N;
size_t start_psi = start_y + N;
size_t start_v = start_psi + N;
size_t start_cte = start_v + N;
size_t start_epsi = start_cte + N;
size_t start_delta = start_epsi + N;
size_t start_acc = start_delta + N - 1;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) {
		this->coeffs = coeffs;
	}

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.


		fg[0] = 0;

		//------Cost Function------//
		// state: [x,y,psi,v,cte,epsi]

		for (uint8_t t = 0; t < N; t++) {
			fg[0] += W_V * CppAD::pow(vars[start_cte + t] - REFERENCE_CTE, 2); // CTE error(0)
			fg[0] += W_EPSI * CppAD::pow(vars[start_epsi + t] - REFERENCE_EPSI,
					2); // Orientation error(0)
			fg[0] += W_CTE * CppAD::pow(vars[start_v + t] - REFERENCE_VELOCITY,
					2); // Velocity error(40)
		}

		// Minimising the use of Actuators, i.e. minimizing the change rate of steer angle and acceleration
		// Add Scaling factor for smoothing the results
		for (uint8_t t = 0; t < N - 1; t++) {
			fg[0] += W_DELTA * CppAD::pow(vars[start_delta + t], 2); // Steer angle
			fg[0] += W_ACC * CppAD::pow(vars[start_acc + t], 2); // Acceleration (throttle)
		}

		// Minimize the value gap between sequential actuations
		// Add Scaling factor for smoothing the results
		for (uint8_t t = 0; t < N - 2; t++) {
			fg[0] += W_DELTA_DOT * CppAD::pow(
					vars[start_delta + t + 1] - vars[start_delta + t], 2);
			fg[0] += W_ACC_DOT * CppAD::pow(
					vars[start_acc + t + 1] - vars[start_acc + t], 2);
		}

		//-----Setting up constrains
		// Initial Constraints

		// as fg[0] is used for cost value, we use 1
		fg[1 + start_x] = vars[start_x];
		fg[1 + start_y] = vars[start_y];
		fg[1 + start_psi] = vars[start_psi];
		fg[1 + start_v] = vars[start_v];
		fg[1 + start_cte] = vars[start_cte];
		fg[1 + start_epsi] = vars[start_epsi];

		// Remaining constraints
		for (uint8_t t = 1; t < N; t++) {
			// state at t + 1
			AD<double> x1 = vars[start_x + t];
			AD<double> y1 = vars[start_y + t];
			AD<double> psi1 = vars[start_psi + t];
			AD<double> v1 = vars[start_v + t];
			AD<double> cte1 = vars[start_cte + t];
			AD<double> epsi1 = vars[start_epsi + t];

			// State at t
			AD<double> x0 = vars[start_x + t - 1];
			AD<double> y0 = vars[start_y + t - 1];
			AD<double> psi0 = vars[start_psi + t - 1];
			AD<double> v0 = vars[start_v + t - 1];
			AD<double> cte0 = vars[start_cte + t - 1];
			AD<double> epsi0 = vars[start_epsi + t - 1];

			// at t, considering the actuations****************
			AD<double> delta0 = vars[start_delta + t - 1];
			AD<double> a0 = vars[start_acc + t - 1];

			// polynomial of degree 3
			AD<double> f0 = 0.0;

			for (int i = 0; i < coeffs.size(); i++) {
				f0 += coeffs[i] * CppAD::pow(x0, i);
			}

			AD<double> psides0 = 0.0;

			for (int i = 1; i < coeffs.size(); i++) {
				psides0 += i * coeffs[i] * CppAD::pow(x0, i - 1);
			}
			psides0 = CppAD::atan(psides0);

			// Equations for the model:
			// x_[t]    = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
			// y_[t]    = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
			// psi_[t]  = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
			// v_[t]    = v[t-1] + a[t-1] * dt
			// cte[t]   = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
			// epsi[t]  = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

			// Constrain to 0
			fg[1 + start_x + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + start_y + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + start_psi + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[1 + start_v + t] = v1 - (v0 + a0 * dt);
			fg[1 + start_cte + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0)
					* dt));
			fg[1 + start_epsi + t] = epsi1 - ((psi0 - psides0) + v0 * delta0
					/ Lf * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {
}
MPC::~MPC() {
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// TODO: Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	//
	// 4 * 10 + 2 * 9

	size_t n_vars = N * 6 + (N - 1) * 2;
	// TODO: Set the number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (uint8_t i = 0; i < n_vars; i++) {
		vars[i] = 0;
	}

	// Setting initial variables

	vars[start_x] = state[0];
	vars[start_y] = state[1];
	vars[start_psi] = state[2];
	vars[start_v] = state[3];
	vars[start_cte] = state[4];
	vars[start_epsi] = state[5];

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// TODO: Set lower and upper limits for variables.

	for (uint8_t i = 0; i < start_delta; i++) {
		vars_lowerbound[i] = -BOUNDS;
		vars_upperbound[i] = BOUNDS;
	}

	// steering angle constrains
	for (uint8_t i = start_delta; i < start_acc; i++) {
		vars_lowerbound[i] = -STEERING_BOUNDS;
		vars_upperbound[i] = STEERING_BOUNDS;
	}

	// acceleration bounds: +/- 1.0
	for (uint8_t i = start_acc; i < n_vars; i++) {
		vars_lowerbound[i] = -ACC_BOUNDS;
		vars_upperbound[i] = ACC_BOUNDS;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);

	for (uint8_t i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	// lowerbound
	constraints_lowerbound[start_x] = state[0];
	constraints_lowerbound[start_y] = state[1];
	constraints_lowerbound[start_psi] = state[2];
	constraints_lowerbound[start_v] = state[3];
	constraints_lowerbound[start_cte] = state[4];
	constraints_lowerbound[start_epsi] = state[5];

	// uppwerbound
	constraints_upperbound[start_x] = state[0];
	constraints_upperbound[start_y] = state[1];
	constraints_upperbound[start_psi] = state[2];
	constraints_upperbound[start_v] = state[3];
	constraints_upperbound[start_cte] = state[4];
	constraints_upperbound[start_epsi] = state[5];

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result < Dvector > solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound,
			vars_upperbound, constraints_lowerbound, constraints_upperbound,
			fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.


	this->res_x = {};
	this->res_y = {};

	for (uint8_t i = 0; i < N; i++) {
		this->res_x.push_back(solution.x[start_x + i]);
		this->res_y.push_back(solution.x[start_y + i]);
	}

	vector<double> result;
	result.push_back(solution.x[start_delta]);
	result.push_back(solution.x[start_acc]);
	return result;
}
