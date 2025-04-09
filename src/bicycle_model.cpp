#include "bicycle_model.hpp"
#include <cmath>

BicycleModel::BicycleModel(double dt, double wheelbase) : dt_(dt), wheelbase_(wheelbase) {}

State BicycleModel::step(const State& s, const Control& u) const {
    State next;

    //Kinematic bicycle model equations
    next.x = s.x + s.v * std::cos(s.theta) * dt_;
    next.y = s.y + s.v * std::sin(s.theta) * dt_;
    next.theta = s.theta + (s.v / wheelbase_) * std::tan(u.delta) * dt_;
    next.v = s.v + u.a * dt_;

    return next;
}

Eigen::VectorXd BicycleModel::toVector(const State& s) const{
    Eigen::VectorXd vec(4);
    vec << s.x, s.y, s.theta, s.v;
    return vec;
}