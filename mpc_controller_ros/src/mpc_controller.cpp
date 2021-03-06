 /*
    @C 2021
    Trong-Doan Nguyen
*/

#include "mpc_controller.h"
//#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>


using CppAD::AD;


class FG_eval 
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        bool _debugging = false;
        int _debug_level= 0; // {0, 1, 2, 3} ~ {no, little, medium, detail}

        double _dt, _ref_cte, _ref_etheta, _ref_vel; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;

        AD<double> cost_cte, cost_etheta, cost_vel;
        
        // Constructor
        FG_eval(Eigen::VectorXd coeffs) 
        { 
            this->coeffs = coeffs; 

            // Set default value  
            _debugging = false;  
            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_etheta  = 0;
            _ref_vel   = 0.5; // m/s
            _w_cte     = 100;
            _w_etheta    = 100;
            _w_vel     = 1;
            _w_angvel   = 100;
            _w_accel   = 50;
            _w_angvel_d = 0;
            _w_accel_d = 0;

            _mpc_steps   = 40;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _cte_start   = _v_start + _mpc_steps;
            _etheta_start  = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params)
        {
            _debugging = params.find("DEBUGGING") != params.end() ? params.at("DEBUGGING") : _debugging;

            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;

            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;

            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_etheta  = params.find("W_EPSI") != params.end()  ? params.at("W_EPSI") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_accel = params.find("W_A") != params.end()     ? params.at("W_A") : _w_accel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_accel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_accel_d;
            
            if (_debugging && (_debug_level >= 3)){
                std::cout << "[MPC::FG_eval::loadParams] mpc_dt        : "   << _dt << endl;

                std::cout << "[MPC::FG_eval::loadParams] mpc_steps     : "   << _mpc_steps << endl;

                std::cout << "[MPC::FG_eval::loadParams] mpc_ref_cte   : "   << _ref_cte << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_ref_vel   : "   << _ref_vel << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_ref_etheta: "   << _ref_etheta << endl;

                std::cout << "[MPC::FG_eval::loadParams] mpc_w_cte      : "  << _w_cte << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_w_etheta   : "  << _w_etheta << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_w_vel      : "  << _w_vel << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc__w_angvel  : "  << _w_angvel << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_w_angvel_d : "  << _w_angvel_d << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_w_accel    : "  << _w_accel << endl;
                std::cout << "[MPC::FG_eval::loadParams] mpc_w_accel_d  : "  << _w_accel_d << endl;
            }
                    
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _cte_start   = _v_start + _mpc_steps;
            _etheta_start  = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
            
            //cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl; 
        }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        // fg: function that evaluates the objective and constraints using the syntax       
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            // fg[0] for cost function
            fg[0] = 0;
            cost_cte =  0;
            cost_etheta = 0;
            cost_vel = 0;

            /*
            for (int i = 0; i < _mpc_steps; i++) 
            {
                cout << i << endl;
                cout << "_x_start" << vars[_x_start + i] <<endl;
                cout << "_y_start" << vars[_y_start + i] <<endl;
                cout << "_theta_start" << vars[_theta_start + i] <<endl;
                cout << "_v_start" << vars[_v_start + i] <<endl;
                cout << "_cte_start" << vars[_cte_start + i] <<endl;
                cout << "_etheta_start" << vars[_etheta_start + i] <<endl;
            }*/

            for (int i = 0; i < _mpc_steps; i++) 
            {
              fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2); // cross deviation error
              fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2); // heading error
              fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2); // speed error

              cost_cte      +=  _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);
              cost_etheta   +=  (_w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2)); 
              cost_vel      +=  (_w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2)); 
            }
            if (_debugging && (_debug_level >= 2)){
                cout << "[MPC::FG_eval] cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta  << ", " << cost_vel << endl;
                if (CppAD::isnan(cost_cte)){
                    std::cout << "params: cte\n";
                    for (int i = 0; i < _mpc_steps; i++){
                        std::cout << vars[_cte_start + i] << ", ";
                    }
                    std::cout << "\n";
                }
                if (CppAD::isnan(cost_etheta)){
                    std::cout << "params: etheta\n";
                    for (int i = 0; i < _mpc_steps; i++){
                        std::cout << vars[_etheta_start + i] << ", ";
                    }
                    std::cout << "\n";
                }
                if (CppAD::isnan(cost_vel)){
                    std::cout << "params: v\n";
                    for (int i = 0; i < _mpc_steps; i++){
                        std::cout << vars[_v_start + i] << ", ";
                    }
                    std::cout << "\n";
                }


            }
            

            // Minimize the use of actuators.
            for (int i = 0; i < _mpc_steps - 1; i++) {
              fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
              fg[0] += _w_accel * CppAD::pow(vars[_a_start + i], 2);
            }
            if (_debugging && (_debug_level >= 2)){
                std::cout << "[MPC::FG_eval] cost of actuators: " << fg[0] << endl;
            }
            

            // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++) {
              fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
              fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
            }
            if (_debugging && (_debug_level >= 2)){
                cout << "[MPC::FG_eval] cost of gap: " << fg[0] << endl; 
            }
            

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _theta_start] = vars[_theta_start];
            fg[1 + _v_start] = vars[_v_start];
            fg[1 + _cte_start] = vars[_cte_start];
            fg[1 + _etheta_start] = vars[_etheta_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];
                AD<double> theta1 = vars[_theta_start + i + 1];
                AD<double> v1 = vars[_v_start + i + 1];
                AD<double> cte1 = vars[_cte_start + i + 1];
                AD<double> etheta1 = vars[_etheta_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];
                AD<double> theta0 = vars[_theta_start + i];
                AD<double> v0 = vars[_v_start + i];
                AD<double> cte0 = vars[_cte_start + i];
                AD<double> etheta0 = vars[_etheta_start + i];

                // Only consider the actuation at time t.
                //AD<double> angvel0 = vars[_angvel_start + i];
                AD<double> w0 = vars[_angvel_start + i];
                AD<double> a0 = vars[_a_start + i];


                //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
                AD<double> f0 = 0.0;
                for (int i = 0; i < coeffs.size(); i++) 
                {
                    f0 += coeffs[i] * CppAD::pow(x0, i);
                }

                //AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
                AD<double> trj_grad0 = 0.0;
                for (int i = 1; i < coeffs.size(); i++) 
                {
                    trj_grad0 += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0)
                }
                trj_grad0 = CppAD::atan(trj_grad0);


                // Here's `x` to get you started.
                // The idea here is to constraint this value to be 0.
                //
                // NOTE: The use of `AD<double>` and use of `CppAD`!
                // This is also CppAD can compute derivatives and pass
                // these to the solver.
                // TODO: Setup the rest of the model constraints
                fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
                fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
                fg[2 + _theta_start + i] = theta1 - (theta0 +  w0 * _dt);
                fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);
                
                fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
                fg[2 + _etheta_start + i] = etheta1 - ((theta0 - trj_grad0) + w0 * _dt);
            }
        }
};


MPC::MPC():
    // Set default value    
    _mpc_steps(20),
    _max_angvel(3.0), // Maximal angvel radian (~30 deg)
    _max_throttle(1.0), // Maximal throttle accel
    _bound_value(1.0e3), // Bound value for other variables
    _x_start     (0),
    n_vars_ (0),
    n_constraints_ (0)

{
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _v_start     = _theta_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;
}

MPC::MPC(const std::map<string, double> &params):
    _params(params),
    _mpc_steps(10),
    _max_angvel(0.5),
    _max_throttle(0.5),
    _bound_value(1.0e3)
{
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _v_start     = _theta_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

    assert(_y_start         > 0);
    assert(_theta_start     > 0);
    assert(_v_start         > 0);
    assert(_cte_start       > 0);
    assert(_etheta_start    > 0);
    assert(_angvel_start    > 0);
    assert(_a_start         > 0);

    if (_params["DEBUGGING"] && (_params["DEBUG_LEVEL"] >= 2)){
        std::cout << "[MPC::loadParams] MPC Obj parameters updated !! " << endl; 
        std::cout << "[MPC::loadParams] mpc_steps       : "  << _mpc_steps << endl;
        std::cout << "[MPC::loadParams] mpc_max_angvel  : "  << _max_angvel << endl;
        std::cout << "[MPC::loadParams] mpc_max_throttle: "  << _max_throttle << endl;
        std::cout << "[MPC::loadParams] mpc_bound_value : "  << _bound_value << endl;
    }
    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    n_vars_ = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    // Set the number of constraints
    n_constraints_ = _mpc_steps * 6;

    mpc_var_init_ = vector<double>(n_vars_);
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x      = state[0];
    const double y      = state[1];
    const double theta  = state[2];
    const double v      = state[3];
    const double cte    = state[4];
    const double etheta = state[5];

    if (_params["DEBUGGING"] && (_params["DEBUG_LEVEL"] >= 2)){
        std::printf("[MPC::Solve] Current state: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n", 
                                    state[0], state[1], state[2], state[3], state[4], state[5]);
        std::printf("[MPC::Solve] Coeff          : (%.3f, %.3f, %.3f %.3f)\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    }

    

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars_);
    // for (int i = 0; i < n_vars_; i++) 
    // {
    //     vars[i] = 0;
    // }

    // Set the initial variable values
    // Warm start 
    vars[_x_start] = x;
    for (int i = _x_start+ 1; i < _y_start - 1; i++){
        vars[i] = mpc_var_init_[i+1];
    }
    vars[_y_start - 1] = vars[_y_start - 2];

    vars[_y_start] = y;
    for (int i = _y_start+ 1; i < _theta_start - 1; i++){
        vars[i] = mpc_var_init_[i+1];
    }
    vars[_theta_start - 1] = vars[_theta_start - 2];

    vars[_theta_start] = theta;
    for (int i = _theta_start+ 1; i < _cte_start - 1; i++){
        vars[i] = mpc_var_init_[i+1];
    }
    vars[_cte_start - 1] = vars[_cte_start - 2];

    vars[_cte_start] = cte;
    for (int i = _cte_start+ 1; i < _etheta_start - 1; i++){
        vars[i] = mpc_var_init_[i+1];
    }
    vars[_etheta_start - 1] = vars[_etheta_start - 2];

    vars[_etheta_start] = etheta;
    for (int i = _etheta_start+ 1; i < n_vars_ - 1; i++){
        vars[i] = mpc_var_init_[i+1];
    }
    vars[n_vars_ - 1] = vars[n_vars_ - 2];




    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars_);
    Dvector vars_upperbound(n_vars_);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }

    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }
    // Acceleration/decceleration upper and lower limits
    for (int i = _a_start; i < n_vars_; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints_);
    Dvector constraints_upperbound(n_constraints_);
    for (int i = 0; i < n_constraints_; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);


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
    // options += "Sparse  true        reverse\n";
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

    if (_params["DEBUGGING"] && (_params["DEBUG_LEVEL"] >= 1)){
        std::cout << "[MPC::Solve] Cost: " << cost << std::endl;
    }

    this->mpc_x = {};
    this->mpc_y = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
    }

    for (int i = 0; i < n_vars_; i++){
        this->mpc_var_init_[i] = solution.x[i];
    }
    

    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}
