#ifndef MULTIAGENTCONTROL_H
#define MULTIAGENTCONTROL_H

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "utilities.h"
#include "articleConstantValues.h"

#define USE_DEFAULT_K true

#define A_DIM 4
#define B_DIM 2

using Point = Eigen::Vector2d;
using Velocity = Eigen::Vector2d;

class MultiAgentControl
{
public:
    MultiAgentControl();
    std::vector<Eigen::Vector2d> getControl(std::vector<Eigen::Vector4d> x, double time, std::vector<Eigen::Vector4d> &h, std::vector<Eigen::Vector4d> &errors);
    Eigen::MatrixXd getWeight(double timeStep){return articleValues->getWeight(timeStep);};
    double getC(int i){return c[i];};

private:
    std::vector<Eigen::Vector4d> getErrors(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, double t);
    Eigen::Vector4d getError(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, int i, double t);
    std::vector<Eigen::Vector2d> nonLinearFunctions(Eigen::MatrixXd B, Eigen::MatrixXd P, std::vector<Eigen::Vector4d> error);
    double rho_i(Eigen::Vector4d error);
    double c_dot_i(Eigen::Vector4d error);

    std::vector<double> c; // c_i = integral of cdot_i

    double timeStep  = 0.01;
    articleConstantValues *articleValues;
};

#endif

