//
// Created by Niraj on 6/23/19.
//

#include "MPC_Kinematic.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using CppAD::AD;


class FG_eval
{
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coefficients;
    MPC_parameters mpc_params_;
    double _Lf, _dt, _ref_cte, _ref_epsi, _ref_vel;
    double  _w_cte, _w_epsi, _w_vel, _w_delta, _w_accel, _w_turn, _w_delta_d, _w_accel_d;
    int _mpc_steps, _x_start, _y_start, _psi_start, _v_start, _cte_start, _epsi_start, _delta_start, _a_start;

    // Constructor
    FG_eval(Eigen::VectorXd coefficients)
    {
        this->coefficients = coefficients;

        // Set default value
        _Lf = 0.25; // distance between the front of the vehicle and its center of gravity
        _dt = 0.1;  // in sec
        _ref_cte   = 0;
        _ref_epsi  = 0;
        _ref_vel   = 1.0; // m/s
        _w_cte     = 100;
        _w_epsi    = 100;
        _w_vel     = 100;
        _w_delta   = 100;
        _w_accel   = 50;
        _w_turn    = 100;
        _w_delta_d = 0;
        _w_accel_d = 0;

        _mpc_steps   = 40;
        _x_start     = 0;
        _y_start     = _x_start + _mpc_steps;
        _psi_start   = _y_start + _mpc_steps;
        _v_start     = _psi_start + _mpc_steps;
        _cte_start   = _v_start + _mpc_steps;
        _epsi_start  = _cte_start + _mpc_steps;
        _delta_start = _epsi_start + _mpc_steps;
        _a_start     = _delta_start + _mpc_steps - 1;
    }

    // Load parameters for constraints
    void SetParameters(MPC_parameters &fg_params)
    {
        mpc_params_ = fg_params;
        _dt = mpc_params_.dt_;
        _Lf = mpc_params_.Lf_;
        _mpc_steps = mpc_params_.mpc_steps_;
        _ref_cte   = mpc_params_.ref_cte_;
        _ref_epsi  = mpc_params_.ref_epsi_;
        _ref_vel   = mpc_params_.ref_vel_;

        _w_cte   = mpc_params_.w_cte_;
        _w_epsi  = mpc_params_.w_epsi_;
        _w_vel   = mpc_params_.w_vel_;
        _w_delta = mpc_params_.w_delta_;
        _w_accel = mpc_params_.w_accel_;
        _w_turn  = mpc_params_.w_turn_;
        _w_delta_d = mpc_params_.w_delta_d_;
        _w_accel_d = mpc_params_.w_accel_d_;

        _x_start     = 0;
        _y_start     = _x_start + _mpc_steps;
        _psi_start   = _y_start + _mpc_steps;
        _v_start     = _psi_start + _mpc_steps;
        _cte_start   = _v_start + _mpc_steps;
        _epsi_start  = _cte_start + _mpc_steps;
        _delta_start = _epsi_start + _mpc_steps;
        _a_start     = _delta_start + _mpc_steps - 1;

//        cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl;
    }

    // MPC implementation (cost func & constraints)
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    // fg: function that evaluates the objective and constraints using the syntax
    void operator()(ADvector& fg, const ADvector& vars)
    {

        // fg[0] for cost function
        fg[0] = 0;
        for (int i = 0; i < _mpc_steps; i++) {
            fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2); // cross deviation error
            fg[0] += _w_epsi * CppAD::pow(vars[_epsi_start + i] - _ref_epsi, 2); // heading error
            fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2); // speed error
        }

        // Minimize the use of actuators.
        for (int i = 0; i < _mpc_steps - 1; i++) {
            fg[0] += _w_delta * CppAD::pow(vars[_delta_start + i], 2);
            fg[0] += _w_accel * CppAD::pow(vars[_a_start + i], 2);
            fg[0] += _w_turn  * CppAD::pow((vars[i + _v_start] * vars[i + _delta_start]), 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < _mpc_steps - 2; i++) {
            fg[0] += _w_delta_d * CppAD::pow(vars[_delta_start + i + 1] - vars[_delta_start + i], 2);
            fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
        }

        // fg[x] for constraints
        // Initial constraints
        fg[1 + _x_start] = vars[_x_start];
        fg[1 + _y_start] = vars[_y_start];
        fg[1 + _psi_start] = vars[_psi_start];
        fg[1 + _v_start] = vars[_v_start];
        fg[1 + _cte_start] = vars[_cte_start];
        fg[1 + _epsi_start] = vars[_epsi_start];

        // Add system dynamic model constraint
        for (int i = 0; i < _mpc_steps - 1; i++)
        {
            // The state at time t+1 .
            AD<double> x1 = vars[_x_start + i + 1];
            AD<double> y1 = vars[_y_start + i + 1];
            AD<double> psi1 = vars[_psi_start + i + 1];
            AD<double> v1 = vars[_v_start + i + 1];
            AD<double> cte1 = vars[_cte_start + i + 1];
            AD<double> epsi1 = vars[_epsi_start + i + 1];

            // The state at time t.
            AD<double> x0 = vars[_x_start + i];
            AD<double> y0 = vars[_y_start + i];
            AD<double> psi0 = vars[_psi_start + i];
            AD<double> v0 = vars[_v_start + i];
            AD<double> cte0 = vars[_cte_start + i];
            AD<double> epsi0 = vars[_epsi_start + i];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[_delta_start + i];
            AD<double> a0 = vars[_a_start + i];

            AD<double> f0 = 0.0;
            for (int j = 0; j < coefficients.size(); j++)
            {
                f0 += coefficients[j] * CppAD::pow(x0, j);
            }
            AD<double> psides0 = 0.0;
            for (int j = 1; j < coefficients.size(); j++)
            {
                psides0 += j*coefficients[j] * CppAD::pow(x0, j-1); // f'(x0)
            }
            psides0 = CppAD::atan(psides0);

            fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * _dt);
            fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * _dt);
            fg[2 + _psi_start + i] = psi1 - (psi0 + v0 * delta0 / _Lf * _dt);
            fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);
            fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * _dt));
            fg[2 + _epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / _Lf * _dt);
        }
    }
};

// ====================================
// MPC class definition implementation.
// ====================================
MPC_Kinematic::MPC_Kinematic()
{
    // Set default value
    _mpc_steps = 20;
    _max_steering = 0.523; // Maximal steering radian (~30 deg)
    _max_throttle = 1.0; // Maximal throttle accel
    _bound_value  = 1.0e3; // Bound value for other variables

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _psi_start   = _y_start + _mpc_steps;
    _v_start     = _psi_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _epsi_start  = _cte_start + _mpc_steps;
    _delta_start = _epsi_start + _mpc_steps;
    _a_start     = _delta_start + _mpc_steps - 1;

}
void MPC_Kinematic::SetInitialParameters(MPC_parameters &params)
{
    mpc_params_ = params;
}

void MPC_Kinematic::CalculateParameters()
{
    _mpc_steps = mpc_params_.mpc_steps_;
    _max_steering = mpc_params_.max_steering_;
    _max_throttle = mpc_params_.max_throttle_;
    _bound_value  = mpc_params_.bound_value_;

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _psi_start   = _y_start + _mpc_steps;
    _v_start     = _psi_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _epsi_start  = _cte_start + _mpc_steps;
    _delta_start = _epsi_start + _mpc_steps;
    _a_start     = _delta_start + _mpc_steps - 1;

    cout << "\n!! MPC Obj parameters updated !! Test value- mpc_steps=" << _mpc_steps<<endl;
}


vector<double> MPC_Kinematic::Solve(Eigen::VectorXd state, Eigen::VectorXd coefficients)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set lower and upper limits for variables.
    for (int i = 0; i < _delta_start; i++)
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = _delta_start; i < _a_start; i++)
    {
        vars_lowerbound[i] = -_max_steering;
        vars_upperbound[i] = _max_steering;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }
    // Velocity upper and lower limits
    for (int i = _v_start; i < _cte_start; i++)
    {
        vars_lowerbound[i] = -mpc_params_.ref_vel_;
        vars_upperbound[i] = mpc_params_.ref_vel_;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_psi_start] = psi;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_epsi_start] = epsi;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_psi_start] = psi;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coefficients);
    fg_eval.SetParameters(mpc_params_);
    // options for IPOPT solver
    std::string options;
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
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    this->mpc_x = {};
    this->mpc_y = {};
    for (int i = 0; i < _mpc_steps; i++)
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
    }
    vector<double> result;
    result.push_back(solution.x[_delta_start]);
    result.push_back(solution.x[_a_start]);
    result.push_back(solution.x[_v_start]);
    return result;
}
