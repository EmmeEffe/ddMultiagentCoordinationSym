/*
 * Library containing all the utilities functions
*/

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector3d;
using Eigen::Vector2d;

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