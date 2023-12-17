#ifndef MULTIAGENTCONTROL_H
#define MULTIAGENTCONTROL_H

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "Riccati_Solver/riccati_solver.h"
#include "utilities.h"

#define USE_DEFAULT_K true

#define A_DIM 4
#define B_DIM 2

using Point = Eigen::Vector2d;
using Velocity = Eigen::Vector2d;

class MultiAgentControl
{
public:
    MultiAgentControl(int _M, int _N, Eigen::VectorXd _b_coeff, Eigen::MatrixXd _a_coeff, std::vector<Vector2d> startPoints, std::vector<Vector2d> endPoints, double maxVelocity, double maxAcceleration);
    std::vector<Eigen::Vector2d> getControl(std::vector<Eigen::Vector4d> x, double time, std::vector<Eigen::Vector4d> &h, std::vector<Eigen::Vector4d> &errors);

private:
    void generateWeightMatrices();
    Eigen::MatrixXi generateSimpleAdjacency(Eigen::VectorXi Followers, Eigen::VectorXi Targets);
    std::vector<Eigen::Vector4d> errors(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h);
    Eigen::Vector4d getError(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, int i);
    std::vector<Eigen::Vector2d> nonLinearFunctions(Eigen::MatrixXd B, Eigen::MatrixXd P, std::vector<Eigen::Vector4d> error, double k);
    Eigen::MatrixXd getK(Eigen::MatrixXd &P);
    Eigen::Vector4d trapezoidalVelocityProfile(Point start, Point end, double maxVelocity, double maxAcceleration, double currentTime);

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd W_Matrix;
    Eigen::VectorXd b_coeff;
    Eigen::MatrixXd a_coeff;
    Eigen::VectorXi Targets;
    Eigen::VectorXi Followers;
    Eigen::MatrixXi adjacency;  // Adjacency Matrix

    // As the matrices don't change, i can define them as private variables
    Eigen::MatrixXd default_P;
    Eigen::MatrixXd default_K;

    std::vector<Eigen::Vector2d> startPoints, endPoints;

    int A_Dim, B_Dim, M, N;
    double maxVelocity, maxAcceleration;

};

#endif

