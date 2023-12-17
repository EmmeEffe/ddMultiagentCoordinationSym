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
            adjacency(Followers(i), Targets(j)) = 1;
        }
    }
    return adjacency;
}

void MultiAgentControl::generateWeightMatrices(){
    W_Matrix = getWeighMatrix(Targets, Followers, adjacency, b_coeff, a_coeff);
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

    errors = this->errors(W_Matrix, x, h);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(A_Dim, A_Dim);
    Eigen::MatrixXd K = this->getK(P);
    std::vector<Eigen::Vector2d> nonLinear = this->nonLinearFunctions(B, P, errors, thresh_k);
    std::vector<Eigen::Vector2d> u(M);
    for(int i=0; i<M; i++){
        u[i] = c * (K * errors[i]) + gamma - mu * nonLinear[i];
    }

    // Limit the magnitude of u
    for(int i=0; i<M; i++){
        normBetween(u[i], sigma);
    }

    return u;
}

std::vector<Eigen::Vector4d> MultiAgentControl::errors(Eigen::MatrixXd weigths, std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h) // Return error array
{
    std::vector<Eigen::Vector4d> errors;

    if( weigths.rows()==N && weigths.cols()==N && h.size() >= (std::size_t)M && x.size() == (std::size_t)N){
        errors.resize(M);
        for(int i=0; i<M; i++){ // For every element in vector
            errors[i] = getError(weigths, x, h, i);
        }
        return errors;
    }else{
        throw std::invalid_argument("Invalid Dimensions");
        return errors;
    }
}

Eigen::Vector4d MultiAgentControl::getError(Eigen::MatrixXd weigths, std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, int i){
    Eigen::Vector4d err(0,0);
    Eigen::Vector4d thisErr = x[i] - h[i];
    for(int j=0; j<M; j++){
        err = err + weigths(i,j) * (thisErr-(x[j] - h[j]));
    }
    for(int k=M; k<N; k++){
        err = err + weigths(i,k) * (thisErr)-x[k];
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