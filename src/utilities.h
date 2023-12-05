/*
 * Library containing all the utilities functions
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Matrix2d;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Quaterniond;

#define DEBUG

/*
 * Return if an element is in vector
*/

bool isInVector(VectorXi vect, int elem){
    for(int i=0; i<vect.size(); i++){
        if(vect(i)==elem)
            return true;
    }
    return false;
}

bool isAdjacent(MatrixXi adjacency, int i, int j){
    int val = adjacency(i,j);
    return val>0;
}

/*
 * Get the weight matrix (formula 1 of the paper)
 *           /
 *          | 0 when i=j or (j, i) not in E
 * w_i_j = <  b_j when j in T (target list) and (j, i) in E
 *          |  a_i_j when both i,j in F and (j, i) in E
 *           \
 * 
 * B has dimension N, A and adjacency have dimension NxN, Targets has dimension M and Followers has dimension N-M
*/

MatrixXf getWeighMatrix(VectorXi Targets, VectorXi Followers, MatrixXi adjacency, VectorXf b_coeff, MatrixXf a_coeff){

    // Verify dimension
    int N = Targets.size() + Followers.size();
    if(!(adjacency.rows()==adjacency.cols()&&adjacency.cols()==N)){
        // TODO Raise an error
    }
    if(!(a_coeff.rows()==a_coeff.cols()&&a_coeff.cols()==N)){
        // TODO Raise an error
    }
    if(!(b_coeff.size()==N)){
        // TODO Raise an error
    }

    MatrixXf weigths(N, N);

    for(int i=0; i<N; i++){
        for(int j=0; j<N; j++){
            if(i==j){                                           // If i==j then zero
                weigths(i,j) = 0;
                continue;
            }
            if(adjacency(j,i)==0){                              // If no edge (j,i) then zero
                weigths(i,j) = 0;
                continue;
            }
            if((isInVector(Targets, j))&&(isAdjacent(adjacency,j,i))){  // If j Target and edge (j,i) then b_j
                weigths(i,j) = (float)b_coeff(j);
                continue;
            }
            if((isInVector(Followers, i))&&(isInVector(Followers, j))&&(isAdjacent(adjacency,j,i))){  // If i,j Followers and edge (j,i) then a_i_j
                weigths(i,j) = a_coeff(i,j);
            }
        }
    }

    #ifdef DEBUG
        std::cout<<"Weights Matrix: "<<weigths;
    #endif

    return weigths;
}

Vector3d xVehicleVersor(float theta){ // Return the versor x in the reference system of the vehicle
    Vector3d versor(cos(theta), sin(theta), 0);
    #ifdef DEBUG
        std::cout<<"VersorX_vehicle: "<<versor<<" Angle theta: "<<theta;
    #endif
    return versor;
}

Vector3d yVehicleVersor(float theta){ // Return the versor y in the reference system of the vehicle
    Vector3d versor((-1)*sin(theta), cos(theta), 0);
    #ifdef DEBUG
        std::cout<<"VersorY_vehicle: "<<versor<<" Angle theta: "<<theta;
    #endif
    return versor;
}

Vector2d velocityToUnicycle(Vector3d velocity, float l, double theta){ // Convert velocity of a point to unicycle inputs
    Vector3d xVers = xVehicleVersor(theta);
    Vector3d yVers = yVehicleVersor(theta);
    double vel_uni = velocity.dot(xVers);
    double theta_dot_uni = velocity.dot(yVers)/l;
    Vector2d unicycle(vel_uni, theta_dot_uni);
    return unicycle;
}

Vector2d velocityToUnicycle(double vel_x, double vel_y, float l, double theta){ // Convert velocity of a point to unicycle inputs
    Vector3d velocity(vel_x, vel_y, 0);
    return velocityToUnicycle(velocity, l, theta);
}

Vector2d polarToCartesian(float len, double angle){  //converts polar to cartesian coordinate
    Vector2d vect(len*cos(angle), len*sin(angle));
    return vect;
}

Vector2d getRobotFormationPosition(Vector2d center, float radius, double start_angle, int i, int N){ // Return the coordinates of the i'th robot in the formation composed by N robots
    Vector2d rotatedVector = polarToCartesian(radius, start_angle + i*(2*M_PI)/N); // Rotate in position
    return rotatedVector + center; // translate
}
Vector2d getRobotFormationPosition(double center_x, double center_y, float radius, double start_angle, int i, int N){ // OVERRIDE
    return getRobotFormationPosition(Vector2d(center_x, center_y), radius, start_angle, i, N);
}

Matrix2d zRotation(double theta){ // Rotation Matrix around z
    return Matrix2d{{cos(theta), (-1)*sin(theta)},{sin(theta), cos(theta)}};
}

Vector2d unicycleToVelocity(double v, double theta_p, double theta, float l){ // Convert unycicle coordinates to velocity of P tilde in fixed reference frame
    Vector2d V_p_tilde_veic(v, l*theta_p);
    return zRotation(theta)*V_p_tilde_veic;
}


double quaternionToTheta(double x, double y, double z, double w){
    x=x+y;
    float yaw = 2.0 * atan2(z,w);
    return yaw;

    //Quaterniond q(w, x, y, z);
    //return quaternionToTheta(q);
}

void normBetween(double &x, double &y, double max){ // saturate between max and -max (in norm)
    if(x*x + y*y > max*max){
        float norm = sqrt(x*x +y*y);
        x/=norm;
        y/=norm;
    }
}

double saturate(double val, double sat){
    return std::max(-sat, std::min(val, sat));
}


#endif