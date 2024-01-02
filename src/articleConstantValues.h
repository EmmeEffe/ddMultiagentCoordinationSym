#ifndef ARTICLECONSTANTVALUES_H
#define ARTICLECONSTANTVALUES_H

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "utilities.h"
#include "Riccati_Solver/riccati_solver.h"

class articleConstantValues
{
public:
    articleConstantValues();

    enum Subgroups{ // Subgroups from the article
        no_subgroup,
        sub_v1,
        sub_v2,
        sub_v3
    };

    MatrixXd getWeight(double time);
    double getWeight(int i, int j, double time);

    // Getter for K, P, rho, B and Gamma
    const Eigen::MatrixXd& getK() const {return K;}
    const Eigen::MatrixXd& getP() const {return P;}
    const Eigen::MatrixXd& getB() const {return B;}
    const Eigen::MatrixXd& getGamma() const {return Gamma;}
    const Eigen::MatrixXd& getRho() const { return rho; }
    Eigen::Vector2d getgamma(double t, int i);
    Eigen::Vector2d calculateGamma(double t, int i, Subgroups subgroup);
    Eigen::Vector4d getprofile(double t, int i);
    Eigen::Vector4d calculateProfile(double t, int i, Subgroups subgroup);
    void addTargetsToPositionList(std::vector<Eigen::Vector4d> &positionList, double time);

    // Inline getters for M, N, mu, sigma, and rho
    int getM() const { return M; }
    int getN() const { return N; }
    double getMu() const { return mu; }
    double getSigma() const { return sigma; }
    Eigen::Vector4d getTargetPosition(int index, double time);

private:
    void resizeAllMatrices();
    void fillMatrices();
    void calcValues();
    void calcK();


    int M, N, stateDim, controlDim;

    // Related to System Dynamics
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd B_tilde;
    Eigen::MatrixXd B_signed;

    // Related to the Riccati Solver
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd K;

    // Related to the gains
    Eigen::MatrixXd Gamma;
    Eigen::MatrixXd rho;
    double mu;
    double sigma;
    double v_max;

    // Related to the graph
    Eigen::MatrixXi adjacency_0; // Before 60s
    Eigen::MatrixXd a_coeffs_0;
    Eigen::VectorXd b_coeffs_0;

    Eigen::MatrixXi adjacency_1; // After 60s
    Eigen::MatrixXd a_coeffs_1;
    Eigen::VectorXd b_coeffs_1;

    Eigen::MatrixXd weights_0;
    Eigen::MatrixXd weights_1;

    Eigen::VectorXi Targets;
    Eigen::VectorXi Followers;
};

#endif