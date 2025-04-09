#include "mpc.hpp"
#include <iostream>

MPC::MPC(const Eigen::MatrixXd& A,
         const Eigen::MatrixXd& B,
         const Eigen::MatrixXd& Q,
         const Eigen::MatrixXd& R,
         int horizon)
    : A_(A), B_(B), Q_(Q), R_(R), horizon_(horizon) {}

Eigen::VectorXd MPC::solveWithDenseQP(
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::VectorXd>& x_ref_traj)
{
    int nx = A_.rows();  // State dimension
    int nu = B_.cols();  // Control dimension
    int N = horizon_;

    // === 1. Build Block Matrices ===
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(N * nx, N * nx);
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N * nu, N * nu);

    for (int i = 0; i < N; ++i) {
        Q_bar.block(i * nx, i * nx, nx, nx) = Q_;
        R_bar.block(i * nu, i * nu, nu, nu) = R_;
    }

    // === 2. Predictive Model Matrices ===
    Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(N * nx, nx);
    Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(N * nx, N * nu);

    for (int i = 0; i < N; ++i) {
        A_bar.block(i * nx, 0, nx, nx) = A_;
        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd AB = A_;
            for (int k = 0; k < i - j; ++k) {
                AB *= A_;
            }
            B_bar.block(i * nx, j * nu, nx, nu) = AB * B_;
        }
    }

    // === 3. Stack reference trajectory ===
    Eigen::VectorXd x_ref_stack(N * nx);
    for (int i = 0; i < N; ++i) {
        x_ref_stack.segment(i * nx, nx) = x_ref_traj[i];
    }

    // === 4. QP Cost ===
    Eigen::MatrixXd H = B_bar.transpose() * Q_bar * B_bar + R_bar;
    Eigen::VectorXd f = B_bar.transpose() * Q_bar * (A_bar * x0 - x_ref_stack);

    // === 5. Solve QP: H * u = -f
    Eigen::VectorXd U = -H.ldlt().solve(f);

    // Return only first control input u_0
    return U.head(nu);
}

