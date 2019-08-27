//
// Created by Niraj on 6/23/19.
//

#ifndef SRC_MPC_KINEMATIC_H
#define SRC_MPC_KINEMATIC_H

#include <vector>
#include <eigen3/Eigen/Core>

using namespace std;

struct MPC_parameters {
    int mpc_steps_;
    int x_start_;
    int y_start_;
    int psi_start_;
    int v_start_;
    int cte_start_;
    int epsi_start_;
    int delta_start_;
    int a_start_;
    double Lf_;
    double dt_;
    double ref_cte_;
    double ref_epsi_;
    double ref_vel_;
    double w_cte_;
    double w_epsi_;
    double w_vel_;
    double w_delta_;
    double w_accel_;
    double w_turn_;
    double w_delta_d_;
    double w_accel_d_;
    double max_steering_;
    double max_throttle_;
    double bound_value_;
};

class MPC_Kinematic {
public:
    MPC_Kinematic();

    vector<double> mpc_x;
    vector<double> mpc_y;
    void SetInitialParameters(MPC_parameters &params);
    void CalculateParameters();

    // Solve the model given an initial state and polynomial coefficients and return the necessary actuation commands.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coefficients);

private:
    // Parameters for mpc solver
    int _mpc_steps, _x_start, _y_start, _psi_start, _v_start, _cte_start, _epsi_start, _delta_start, _a_start;
    double _max_steering, _max_throttle, _bound_value;
    MPC_parameters mpc_params_;

};


#endif //SRC_MPC_KINEMATIC_H
