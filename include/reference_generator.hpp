#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

class ReferenceGenerator{
    public:
        ReferenceGenerator(double dt, int horizon, const std::string& mode = "circle");

        std::vector<Eigen::VectorXd> generateTrajectory(int step_start) const;
        Eigen::VectorXd getReferenceAt(int step) const;
        std::vector<Eigen::VectorXd> getReferenceHorizon(int step) const;


    private:
        double dt_;
        int horizon_;
        std::string mode_;

        Eigen::VectorXd computeRefState(int step) const;

};

