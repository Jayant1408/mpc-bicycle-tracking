#include "reference_generator.hpp"
#include <cmath>

ReferenceGenerator::ReferenceGenerator(double dt, int horizon, const std::string& mode)
    : dt_(dt), horizon_(horizon), mode_(mode) {}

std::vector<Eigen::VectorXd> ReferenceGenerator::generateTrajectory(int step_start) const {
    std::vector<Eigen::VectorXd> x_refs;
    for (int i = 0; i < horizon_; ++i) {
        x_refs.push_back(computeRefState(step_start + i));
    }
    return x_refs;
}

Eigen::VectorXd ReferenceGenerator::getReferenceAt(int step) const {
    return computeRefState(step);
}

std::vector<Eigen::VectorXd> ReferenceGenerator::getReferenceHorizon(int step) const {
    std::vector<Eigen::VectorXd> horizon;
    for (int i = 0; i < horizon_; ++i) {
        Eigen::VectorXd ref = getReferenceAt(step + i);
        horizon.push_back(ref);
    }
    return horizon;
}

Eigen::VectorXd ReferenceGenerator::computeRefState(int step) const {
    Eigen::VectorXd ref(4); // x, y, theta, v
    double t = step * dt_;

    if (mode_ == "circle") {
        ref(0) = 5.0 * cos(0.2 * t);                             // x
        ref(1) = 5.0 * sin(0.2 * t);                             // y
        ref(2) = atan2(-sin(0.2 * t), -cos(0.2 * t));            // theta (tangent direction)
        ref(3) = 1.0;                                            // constant reference speed
    }
    else if (mode_ == "sine") {
        ref(0) = t;                                              // x
        ref(1) = 2.0 * sin(0.3 * t);                             // y
        ref(2) = atan2(0.6 * cos(0.3 * t), 1.0);                 // theta
        ref(3) = 1.0;                                            // constant reference speed
    }
    else {
        ref << t, 0.0, 0.0, 1.0;                                 // straight-line default
    }

    return ref;
}
