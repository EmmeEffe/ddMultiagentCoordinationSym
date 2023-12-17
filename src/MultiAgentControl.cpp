#include "MultiAgentControl.h"

MultiAgentControl::MultiAgentControl(int _M, int _N, Eigen::VectorXd _b_coeff, Eigen::MatrixXd _a_coeff, std::vector<Vector2d> startPoints, std::vector<Vector2d> endPoints, double maxVelocity, double maxAcceleration)
{
    A_Dim = A_DIM;
    B_Dim = B_DIM;
    A = Eigen::MatrixXd::Zero(A_Dim, A_Dim);
    B = Eigen::MatrixXd::Zero(A_Dim, B_Dim);
    default_P = Eigen::MatrixXd::Zero(A_Dim, A_Dim);
    default_K = Eigen::MatrixXd::Zero(B_Dim, A_Dim);

    // Custom Values from the paper
    A << 0,1,0,0,
         0,0,0,0,
         0,0,0,1,
         0,0,0,0;
    B << 0,0,
         1,0,
         0,0,
         0,1;

    double sqrt_3 = sqrt(3);

    // PreCalculated Values from the paper

    default_P << sqrt_3, 1, 0, 0,
                 1, sqrt_3, 0, 0,
                 0, 0, sqrt_3, 1,
                 0, 0, 1, sqrt_3;

    default_K << -1, -sqrt_3, 0, 0,
                 0, 0, -1, -sqrt_3;

    M = _M;
    N = _N;
    b_coeff = _b_coeff;
    a_coeff = _a_coeff;
    this->maxVelocity = maxVelocity;
    this->maxAcceleration = maxAcceleration;
    this->startPoints = startPoints;
    this->endPoints = endPoints;
    // For Simple Usage i define the Follower vector as the first M elements and the Target vector as the last N-M elements
    Followers.resize(M);
    Targets.resize(N-M);
    for(int i=0; i<M; i++){
        Followers(i) = i;
    }
    for(int i=0; i<N-M; i++){
        Targets(i) = i+M;
    }

    // For Simple Usage i define the Adjacency Matrix that has every follower connected to every target
    adjacency = generateSimpleAdjacency(Followers, Targets);
    generateWeightMatrices();
}

Eigen::MatrixXi MultiAgentControl::generateSimpleAdjacency(Eigen::VectorXi Followers, Eigen::VectorXi Targets){
    Eigen::MatrixXi adjacency = Eigen::MatrixXi::Zero(N, N);
    for(int i=0; i<M; i++){
        for(int j=0; j<N-M; j++){
            adjacency(Targets(j), Followers(i)) = 1;
        }
    }
    return adjacency;
}

void MultiAgentControl::generateWeightMatrices(){
    W_Matrix = getWeighMatrix(Targets, Followers, adjacency, b_coeff, a_coeff);
    std::cout << "W_Matrix in generation:\n" << W_Matrix << std::endl;
    std::cout << "b_coeff: " << b_coeff << std::endl;
    std::cout << "a_coeff: " << a_coeff << std::endl;
    std::cout << "Followers: " << Followers << std::endl;
    std::cout << "Targets: " << Targets << std::endl;
    std::cout << "adjacency: " << adjacency << std::endl;
}

std::vector<Eigen::Vector2d> MultiAgentControl::getControl(std::vector<Eigen::Vector4d> x, double time, std::vector<Eigen::Vector4d> &h, std::vector<Eigen::Vector4d> &errors){
    // u_i = cK*errors + gamma - mu*nonLinear(error)

    // Parameters
    /*
     * c > 0
     * sigma = Saturation value for u_k(t)
     * mu > sigma >= 0
     * thresh_k > 0 is to avoid chattering
    */

    double thresh_k = 0.1; // TODO Move from here
    double c = 1; // TODO Move from here
    double mu = 2; // TODO Move from here
    double sigma = 2; // TODO Move from here
    Eigen::Vector2d gamma(0,0); // TODO FIGURE OUT WHAT IS GAMMA

    h.resize(M);
    errors.resize(M);

    for(int i=0; i<M; i++){
        h[i] = trapezoidalVelocityProfile(startPoints[i], endPoints[i], maxVelocity, maxAcceleration, time);
    }
    errors = this->errors(x, h);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A_Dim, A_Dim);
    Eigen::MatrixXd K = this->getK(P);
    std::vector<Eigen::Vector2d> nonLinear = this->nonLinearFunctions(B, P, errors, thresh_k);
    std::vector<Eigen::Vector2d> u(M);
    for(int i=0; i<M; i++){
        u[i] = c * (K * errors[i]) + gamma - mu * nonLinear[i];

        // Check if u is nan
        if(std::isnan(u[i](0)) || std::isnan(u[i](1))){
        throw std::runtime_error("Error value is NaN");
        }

        normBetween(u[i], sigma);

        // Check if u is nan after normBetween
        if(std::isnan(u[i](0)) || std::isnan(u[i](1))){
        throw std::runtime_error("Error value is NaN");
        }
    }

    return u;
}

std::vector<Eigen::Vector4d> MultiAgentControl::errors(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h) // Return error array
{
    std::vector<Eigen::Vector4d> errors;

    if( W_Matrix.rows()==N && W_Matrix.cols()==N && h.size() >= (std::size_t)M && x.size() == (std::size_t)N){
        errors.resize(M);
        for(int i=0; i<M; i++){ // For every element in vector
            errors[i] = getError(x, h, i);
        }
        return errors;
    }else{
        throw std::invalid_argument("Invalid Dimensions");
        return errors;
    }
}

Eigen::Vector4d MultiAgentControl::getError(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, int i){
    Eigen::Vector4d err(0,0, 0, 0);
    Eigen::Vector4d thisErr = x[i] - h[i];
    for(int j=0; j<M; j++){
        err = err + W_Matrix(i,j) * (thisErr-(x[j] - h[j]));
        // Raise error and print variables
    if (std::isnan(err.x()) || std::isnan(err.y()) || std::isnan(err.z()) || std::isnan(err.w())) {
        std::cout << "Error value is NaN -- INSIDE FIRST" << std::endl;
        std::cout << err.transpose() << std::endl;
        std::cout << "j" << j << std::endl;
        std::cout << "x[" << i << "]: " << x[i].transpose() << std::endl;
        std::cout << "h[" << i << "]: " << h[i].transpose() << std::endl;
        std::cout << "thisErr: " << thisErr.transpose() << std::endl;
        std::cout << "W_Matrix.row(" << i << "): " << W_Matrix.row(i) << std::endl;
        std::cout << "W_Matrix.col(" << i << "): " << W_Matrix.col(i) << std::endl;
        std::cout << "W_Matrix: " << std::endl << W_Matrix << std::endl;
        throw std::runtime_error("Error value is NaN");
    }
    }
    for(int k=M; k<N; k++){
        err = err + W_Matrix(i,k) * (thisErr-x[k]);    
        if (std::isnan(err.x()) || std::isnan(err.y()) || std::isnan(err.z()) || std::isnan(err.w())) {
        std::cout << "Error value is NaN -- INSIDE SECOND" << std::endl;
        std::cout << err.transpose() << std::endl;
        std::cout << "k" << k << std::endl;
        std::cout << "x[" << i << "]: " << x[i].transpose() << std::endl;
        std::cout << "h[" << i << "]: " << h[i].transpose() << std::endl;
        std::cout << "thisErr: " << thisErr.transpose() << std::endl;
        std::cout << "W_Matrix.row(" << i << "): " << W_Matrix.row(i) << std::endl;
        std::cout << "W_Matrix.col(" << i << "): " << W_Matrix.col(i) << std::endl;
        std::cout << "W_Matrix: " << std::endl << W_Matrix << std::endl;
        throw std::runtime_error("Error value is NaN");
    }
    }

    // Raise error and print variables
    if (std::isnan(err.x()) || std::isnan(err.y()) || std::isnan(err.z()) || std::isnan(err.w())) {
        std::cout << "Error value is NaN" << std::endl;
        std::cout << err.transpose() << std::endl;
        std::cout << "x[" << i << "]: " << x[i].transpose() << std::endl;
        std::cout << "h[" << i << "]: " << h[i].transpose() << std::endl;
        std::cout << "thisErr: " << thisErr.transpose() << std::endl;
        std::cout << "W_Matrix.row(" << i << "): " << W_Matrix.row(i) << std::endl;
        std::cout << "W_Matrix.col(" << i << "): " << W_Matrix.col(i) << std::endl;
        std::cout << "W_Matrix: " << std::endl << W_Matrix << std::endl;
        throw std::runtime_error("Error value is NaN");
    }

    return err;
}

std::vector<Eigen::Vector2d> MultiAgentControl::nonLinearFunctions(Eigen::MatrixXd B, Eigen::MatrixXd P, std::vector<Eigen::Vector4d> error, double k) {
    // Check dimensions
    if (B.rows() != A_Dim || B.cols() != B_Dim) {
        throw std::invalid_argument("Dimensions of B do not match the expected dimensions");
    }
    if (P.rows() != 4) {
        throw std::invalid_argument("Number of rows in P must be equal to size of status vector");
    }

    std::vector<Eigen::Vector2d> nonLinear;
    nonLinear.resize(M);
    for(int i=0; i<M; i++){
        Eigen::Vector2d val = B.transpose() * (P * error[i]);
        double norm = val.norm();
        if(norm>k){
            nonLinear[i] = val/norm;
        }else{
            nonLinear[i] = val/k;
        }
        if (std::isnan(nonLinear[i].x()) || std::isnan(nonLinear[i].y())) {
            throw std::runtime_error("Nonlinear value is NaN");
        }
    }
    return nonLinear;
}


Eigen::MatrixXd MultiAgentControl::getK(Eigen::MatrixXd &P){ // Return K matrix

    if(USE_DEFAULT_K){
        P = default_P;
        return default_K;
    }
    /*
        * A 4x4
        * B 4x2
        * Q 4x4
        * R 2x2
        * P 4x4
        * K 2x4
    */

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(A_Dim, A_Dim);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(B_Dim, B_Dim);

    std::cout << "A_Dim" << A_Dim << std::endl;
    std::cout << "B_Dim" << B_Dim << std::endl;
    std::cout << "A:\n" << A << std::endl;
    std::cout << "B:\n" << B << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "R:\n" << R << std::endl;

    solveRiccatiIterationC(A, B, Q, R, P); // Solve and calc P
    std::cout << "P:\n" << P << std::endl;

    Eigen::MatrixXd K = -R.inverse() * (B.transpose()*P);
    return K;
}

Eigen::Vector4d MultiAgentControl::trapezoidalVelocityProfile(Point start, Point end, double maxVelocity, double maxAcceleration, double currentTime) {
    Point dist_vect = end - start;

    double distance = dist_vect.norm();
    double fact_x = dist_vect(0) / distance;
    double fact_y = dist_vect(1) / distance;
    double timeToMaxVelocity = maxVelocity / maxAcceleration;
    double distanceToMaxVelocity = 0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;

    if(distance < 2 * distanceToMaxVelocity){
        maxVelocity = sqrt(distance * maxAcceleration);
        timeToMaxVelocity = maxVelocity / maxAcceleration;
        distanceToMaxVelocity = 0.5 * maxAcceleration * timeToMaxVelocity * timeToMaxVelocity;
    }

    double time_max_velocity = (distance - 2 * distanceToMaxVelocity) / maxVelocity;
    double timeToMaxVelocity2 = timeToMaxVelocity + time_max_velocity;

    if(currentTime<timeToMaxVelocity){
        return Eigen::Vector4d(start(0) + fact_x * maxAcceleration * currentTime * currentTime / 2,
                                fact_x * maxAcceleration * currentTime,
                                start(1) + fact_y * maxAcceleration * currentTime * currentTime / 2,
                                fact_y * maxAcceleration * currentTime);
    }else if(currentTime<timeToMaxVelocity2){
        return Eigen::Vector4d(start(0) + fact_x * (distanceToMaxVelocity + maxVelocity * (currentTime - timeToMaxVelocity)),
                                fact_x * maxVelocity,
                                start(1) + fact_y * (distanceToMaxVelocity + maxVelocity * (currentTime - timeToMaxVelocity)),
                                fact_y * maxVelocity);
    }else if(currentTime<timeToMaxVelocity2+timeToMaxVelocity){
        double time = currentTime - timeToMaxVelocity2;
        double res_time = timeToMaxVelocity+timeToMaxVelocity2 - currentTime;
        return Eigen::Vector4d(start(0) + fact_x * (distance - maxAcceleration * res_time * res_time / 2),
                                fact_x * (maxVelocity -maxAcceleration * time),
                                start(1) + fact_y * (distance - maxAcceleration * res_time * res_time / 2),
                                fact_y * (maxVelocity -maxAcceleration * time));
    }
    else{
        return Eigen::Vector4d(end(0), 0, end(1), 0);
    }
}