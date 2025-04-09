#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "bicycle_model.hpp"
#include "reference_generator.hpp"
#include "mpc.hpp"


YAML::Node loadConfig(const std::string& path)
{
    try{
        return YAML::LoadFile(path);
    } catch (const std::exception& e){
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        exit(EXIT_FAILURE); 
    }
}

int main()
{   
    YAML::Node config = loadConfig("config.yaml");

    const double dt = config["simulation"]["dt"].as<double>();
    const double L = config["simulation"]["L"].as<double>();
    const int total_steps = config["simulation"]["total_steps"].as<int>();
    const int prediction_horizon = config["simulation"]["prediction_horizon"].as<int>();
    const std::string mode = config["trajectory_mode"].as<std::string>();

    State x;
    x.x = config["initial_state"]["x"].as<double>();
    x.y = config["initial_state"]["y"].as<double>();
    x.theta = config["initial_state"]["theta"].as<double>();
    x.v = config["initial_state"]["v"].as<double>();


    Eigen::VectorXd q_vec(4);
    Eigen::VectorXd r_vec(2);

    for (int i = 0; i < 4; ++i) q_vec(i) = config["cost"]["Q"][i].as<double>();
    for (int i = 0; i < 2; ++i) r_vec(i) = config["cost"]["R"][i].as<double>();

    Eigen::MatrixXd Q = q_vec.asDiagonal();
    Eigen::MatrixXd R = r_vec.asDiagonal();

    BicycleModel model(dt, L);
    ReferenceGenerator ref_gen(dt, prediction_horizon, mode);

    // const double dt = 0.1;                 // Time step
    // const double L = 2.5;                  // Wheelbase
    // const int total_steps = 150;           // More steps to let MPC converge
    // const int prediction_horizon = 30;     // Longer prediction horizon

    // // Initialize model and reference generator
    // BicycleModel model(dt, L);
    // ReferenceGenerator ref_gen(dt, prediction_horizon, "sine");  // or "circle" later
    // // ReferenceGenerator ref_gen(dt, prediction_horizon, "circle");  // or "circle" later


    // // Initial state with small velocity
    // State x = {0.0, 0.0, 0.0, 1.0};

    // Discrete-time linearized A and B
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(4, 4);
    A(0, 2) = dt;
    A(1, 3) = dt;

    Eigen::MatrixXd B(4, 2);
    B << 0, 0,
         0, 0,
         dt, 0,
         0, dt;

    // Cost matrices (weights)
    // Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    // Q.diagonal() << 200.0, 200.0, 20.0, 1.0;

    // Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    // R.diagonal() << 0.5, 1.0;
    
    //     // Cost matrices (tuned)
    // Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    // Q.diagonal() << 250.0, 250.0, 50.0, 1.0;   // prioritize tighter heading + position

    // Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    // R.diagonal() << 0.2, 0.5;                  // allow slightly more input flexibility


    MPC mpc(A, B, Q, R, prediction_horizon);

    // Open log
    std::ofstream log("data/bicycle_model_log.csv");
    if (!log.is_open()) {
        std::cerr << "❌ Failed to open log file for writing! Ensure the 'data/' directory exists.\n";
        return -1;
    }

    // CSV headers
    log << "step,x,y,theta,v,x_ref,y_ref,theta_ref,v_ref,a_cmd,delta_cmd,x_err,y_err,theta_err,v_err\n";

    for (int step = 0; step <= total_steps; ++step)
    {
        // Reference trajectory
        std::vector<Eigen::VectorXd> x_refs = ref_gen.getReferenceHorizon(step);
        const Eigen::VectorXd& ref_now = x_refs[0];

        // Current state as vector
        Eigen::VectorXd x_vec(4);
        x_vec << x.x, x.y, x.theta, x.v;

        // Solve MPC
        Eigen::VectorXd u = mpc.solveWithDenseQP(x_vec, x_refs);

        // Clamp control inputs for stability
        u(0) = std::clamp(u(0), -3.0, 3.0);      // acceleration
        u(1) = std::clamp(u(1), -0.5, 0.5);      // steering angle

        Control u_struct = {u(0), u(1)};
        x = model.step(x, u_struct);  // Simulate

        // Compute error
        double x_err = x.x - ref_now(0);
        double y_err = x.y - ref_now(1);
        double theta_err = x.theta - ref_now(2);
        double v_err = x.v - ref_now(3);

        // Log everything
        log << step << "," << x.x << "," << x.y << "," << x.theta << "," << x.v << ","
            << ref_now(0) << "," << ref_now(1) << "," << ref_now(2) << "," << ref_now(3) << ","
            << u(0) << "," << u(1) << ","
            << x_err << "," << y_err << "," << theta_err << "," << v_err << "\n";
    }

    log.close();
    std::cout << "✅ Bicycle model simulation complete.\n";
    return 0;
}








///////////////////////////////////
// #include <iostream>
// #include <fstream>
// #include <cmath>
// #include <Eigen/Dense>
// #include "bicycle_model.hpp"
// #include "reference_generator.hpp"
// #include "mpc.hpp"

// int main()
// {
//     const double dt = 0.1;      // Time step
//     const double L = 2.5;       // Wheelbase
//     const int total_steps = 100;
//     // const int prediction_horizon = 10;
//     const int prediction_horizon = 30;  // Was 20

//     // Initialize model and reference generator
//     BicycleModel model(dt, L);
//     ReferenceGenerator ref_gen(dt, prediction_horizon, "circle");

//     // Initial state
//     State x = {0.0, 0.0, 0.0, 0.0};

//     Eigen::MatrixXd A = Eigen::MatrixXd::Identity(4,4);

//     A(0,2) = dt;
//     A(1,3) = dt;

//     Eigen::MatrixXd B(4,2);
//     B << 0,0,
//          0,0,
//          dt,0,
//          0,dt;

//     // Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4,4);
//     // Q(0,0) = 20.0;  // x position
//     // Q(1,1) = 20.0;  // y position
//     // Q(2,2) = 1.0;   // heading angle
//     // Q(3,3) = 1.0;   // velocity
//     // // Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(2,2);
//     // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);
//     // R(0,0) = 0.5;  // acceleration effort
//     // R(1,1) = 0.2;  // steering effort
//     // const int prediction_horizon = 20; // Extended lookahead

//     // Strong positional accuracy
//     Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
//     Q(0,0) = 100.0;  // x
//     Q(1,1) = 100.0;  // y
//     Q(2,2) = 10.0;   // theta
//     Q(3,3) = 1.0;    // v

//     // Penalize sharp control
//     Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
//     R(0,0) = 5.0;    // acceleration
//     R(1,1) = 10.0;   // steering angle

//     MPC mpc(A,B,Q,R,prediction_horizon);
   


//     std::ofstream log("data/bicycle_model_log.csv");
//     if (!log.is_open()) {
//         std::cerr << "❌ Failed to open log file for writing! Ensure the 'data/' directory exists.\n";
//         return 1;
//     }

//     log << "step,x,y,theta,v,x_ref,y_ref,theta_ref,v_ref,a_cmd,delta_cmd,x_err,y_err,theta_err,v_err\n";


//     for (int step = 0; step <= total_steps; ++step)
//     {
//         std::vector<Eigen::VectorXd> x_refs = ref_gen.getReferenceHorizon(step);

//         Eigen::VectorXd x_vec(4);
//         x_vec << x.x, x.y, x.theta, x.v;

//         Eigen::VectorXd u = mpc.solveWithDenseQP(x_vec, x_refs);
//         Control u_struct = {u(0), u(1)};

//         x = model.step(x, u_struct);

//         const Eigen::VectorXd& ref_now = x_refs[0];
//         double x_err = x.x - ref_now(0);
//         double y_err = x.y - ref_now(1);
//         double theta_err = x.theta - ref_now(2);
//         double v_err = x.v - ref_now(3);

//         log << step << "," << x.x << "," << x.y << "," << x.theta << "," << x.v << ","
//             << ref_now(0) << "," << ref_now(1) << "," << ref_now(2) << "," << ref_now(3) << ","
//             << u(0) << "," << u(1) << ","
//             << x_err << "," << y_err << "," << theta_err << "," << v_err << "\n";
//     }

//     log.close();
//     std::cout << "✅ Bicycle model simulation complete." << std::endl;
//     return 0;
// }
