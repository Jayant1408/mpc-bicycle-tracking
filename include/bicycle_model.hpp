#pragma once
#include <Eigen/Dense>

//State: [x,y,theta,v]
//Control: [acceleration (a), steering angle (delta)]

struct State{
    double x; // Position in X
    double y; // Position in Y
    double theta; // Heading Angle
    double v; // Linear Velocity
};

struct Control{
    double a; // Acceleration
    double delta; //Steering angle
};

class BicycleModel{
    public:
        BicycleModel(double dt, double wheelbase);

        State step(const State& current, const Control& u) const;
        Eigen::VectorXd toVector(const State& s) const;
    private:
        double dt_; //Time step
        double wheelbase_; // Distance between front and rear axle
        
};


