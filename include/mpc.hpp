#pragma once

#include <Eigen/Dense>
#include <vector>

class MPC {
public:
    MPC(const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        int horizon);

    // Solve using dense QP (Eigen-based)
    Eigen::VectorXd solveWithDenseQP(
        const Eigen::VectorXd& x0,
        const std::vector<Eigen::VectorXd>& x_ref_traj);

    // Solve using OSQP (sparse QP)
    Eigen::VectorXd solveWithOSQP(
        const Eigen::VectorXd& x0,
        const std::vector<Eigen::VectorXd>& x_ref_traj);

private:
    Eigen::MatrixXd A_, B_, Q_, R_;
    int horizon_;
};
