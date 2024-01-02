#include "articleConstantValues.h"

articleConstantValues::articleConstantValues()
{
    resizeAllMatrices();
    fillMatrices();
    calcValues();
}

MatrixXd articleConstantValues::getWeight(double time) // Return the weight matrix based on the time
{
    if(time<60)
        return weights_0;
    else
        return weights_1;
}

double articleConstantValues::getWeight(int i, int j, double time) // Return the weight values based on the time
{
    if(time<60)
        return weights_0(i,j);
    else
        return weights_1(i,j);
}

void articleConstantValues::resizeAllMatrices()
{
    // Populate all the matrices and values

    M = 12;
    N = 16;
    stateDim = 4;
    controlDim = 2;

    // Resize all the matrices and the vectors

    A.resize(stateDim, stateDim);
    B.resize(stateDim, controlDim);
    B_tilde.resize(controlDim, stateDim);
    B_signed.resize(controlDim, stateDim);
    Q.resize(stateDim, stateDim);
    R.resize(controlDim, controlDim);
    P.resize(stateDim, stateDim);
    K.resize(controlDim, stateDim);
    Gamma.resize(stateDim, stateDim);
    rho.resize(stateDim, stateDim);

    adjacency_0.resize(N, N);
    a_coeffs_0.resize(M, M);
    b_coeffs_0.resize(N-M);
    adjacency_1.resize(N, N);
    a_coeffs_1.resize(M, M);
    b_coeffs_1.resize(N-M);

    weights_0.resize(N, N);
    weights_1.resize(N, N);

    Targets.resize(N-M);
    Followers.resize(M);
}

void articleConstantValues::fillMatrices()
{
    A << 0, 0, 1, 0,
         0, 0, 0, 0,
         0, 0, 0, 1,
         0, 0, 0, 0;

    B << 0, 0,
        1, 0,
        0, 0,
        0, 1;

    B_tilde << 0, 1, 0, 0,
                0, 0, 0, 1;

    B_signed << 1, 0, 0, 0,
                0, 0, 1, 0;

    Q << 0.05, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.2, 0,
        0, 0, 0, 0.01;

    R = Eigen::MatrixXd::Identity(controlDim, controlDim);

    P << 0.1512, 0.2236, 0, 0,
        0.2236, 0.6762, 0, 0,
        0, 0, 0.4253, 0.4472,
        0, 0, 0.4472, 0.9510;

    K << -0.2236, -0.6762, 0, 0,
        0, 0, -0.4472, -0.9510;

    Gamma << 0.0500, 0.1512, 0, 0,
        0.1512, 0.4572, 0, 0,
        0, 0, 0.2000, 0.4253,
        0, 0, 0.4253, 0.9044;

    rho << 0.1512, 0.2236, 0, 0,
        0.2236, 0.6762, 0, 0,
        0, 0, 0.4253, 0.4472,
        0, 0, 0.4472, 0.9510;

               //   1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16
    adjacency_0 <<  0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1
                    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2
                    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, // 3
                    1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5
                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, // 6
                    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, // 7
                    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 8
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, // 9
                    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, // 10
                    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 11
                    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 12
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 13
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 14
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 15
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // 16

               //   1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12
    a_coeffs_0 <<   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 1
                    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2
                    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, // 3
                    1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5
                    0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, // 6
                    0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 3, // 7
                    0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, // 8
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, // 9
                    0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1, 0, // 10
                    0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, // 11
                    0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0;  // 12

    b_coeffs_0 << 2, 1, 1, 3;

    // After 60s

                //  1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16
    adjacency_1 <<  0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1
                    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2
                    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, // 3
                    0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, // 4
                    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5
                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, // 6
                    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, // 7
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, // 8
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 9
                    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, // 10
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, // 11
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, // 12
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 13
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 14
                    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 15
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; // 16

               //   1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12
    a_coeffs_1 <<   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 1
                    1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,  // 2
                    0, 0, 0, 0, 0, 0, 0, 0,-1, 0, 0, 0,  // 3
                    0, 2, 1, 0,-2, 0, 0, 0, 1, 0, 0, 0,  // 4
                    0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0,  // 5
                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,  // 6
                    0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 2, 0,  // 7
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-2, 0,  // 8
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 9
                    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,  // 10
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2,  // 11
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0;  // 12

    b_coeffs_1 << 3, 2, 2, 3;

    mu = 1;
    sigma = 0.5;
    v_max = 10; // 10m/s

    Followers << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11; // IMPORTANT, Index from zero
    Targets << 12, 13, 14, 15;
}

void articleConstantValues::calcValues()
{
    // Calculate the values that can be calculated from the matrices

    // Calculate the weights
    weights_0 = getWeighMatrix(Targets, Followers, adjacency_0, b_coeffs_0, a_coeffs_0.transpose());
    weights_1 = getWeighMatrix(Targets, Followers, adjacency_1, b_coeffs_1, a_coeffs_1.transpose());

}

void articleConstantValues::calcK() // If you want to manually calculate K and P
{
    solveRiccatiIterationC(A, B, Q, R, P); // Solve and calc P
    Eigen::MatrixXd K = -R.inverse().eval() * (B.transpose().eval()*P);
}

void articleConstantValues::addTargetsToPositionList(std::vector<Eigen::Vector4d> &positionList, double time)
{
    // Add the targets to the position list
    positionList.resize(N);
    for(int i=M; i<N; i++){
        positionList[i] = getTargetPosition(i, time);
    }
}

Eigen::Vector4d articleConstantValues::getTargetPosition(int index, double time)
{
    Eigen::Vector4d target_pos; // p_x, v_x, p_y, v_y
    if(time<60){ // All targets grouped
        // all target close to (40,40)
        switch(index){
            case 12:
                target_pos = Eigen::Vector4d(38/60*time, 38/60, 38/60*time, 38/60);
            break;
            case 13:
                target_pos = Eigen::Vector4d(42/60*time, 42/60, 38/60*time, 38/60);
            break;
            case 14:
                target_pos = Eigen::Vector4d(42/60*time, 42/60, 42/60*time, 42/60);
            break;
            case 15:
                target_pos = Eigen::Vector4d(38/60*time, 38/60, 42/60*time, 42/60);
            break;
        }
    }else{ // target sparse
        switch(index){
            case 12:
                target_pos = Eigen::Vector4d(68, 0, 222, 0);
            break;
            case 13:
                target_pos = Eigen::Vector4d(72, 0, 218, 0);
            break;
            case 14:
                target_pos = Eigen::Vector4d(160, 0, 160, 0);
            break;
            case 15:
                target_pos = Eigen::Vector4d(210, 0, 65, 0);
            break;
        }
    }
    return target_pos;
}

Eigen::Vector2d articleConstantValues::getgamma(double t, int i){
    if(t<60){
        return calculateGamma(t,i, no_subgroup);
    }else{
        switch(i){
            case 0:
            case 1:
            case 2:
            case 3:
                return calculateGamma(t,i,sub_v1);
            break;
            case 4:
            case 5:
            case 6:
            case 7:
                return calculateGamma(t,i,sub_v2);
            break;
            case 8:
            case 9:
            case 10:
            case 11:
                return calculateGamma(t,i,sub_v3);
            break;
            default:
                return Eigen::Vector2d(0,0); // error
        }
    }
}

Eigen::Vector2d articleConstantValues::calculateGamma(double t, int i, Subgroups subgroup)
{
    double omega, r;
    int total_num;
    Eigen::Vector2d gamma(0,0);
    switch(subgroup){
        case no_subgroup:
            omega = 0.1;
            r = 2;
            total_num = 12;
        break;
        case sub_v1:
            omega = 0.1;
            r = 2;
            total_num = 4;
        break;
        case sub_v2:
            omega = 0.2;
            r = 3;
            total_num = 4;
        break;
        case sub_v3:
            omega = 0.3;
            r = 3;
            total_num = 4;
        break;
        default: // Error
            omega = 0;
            r = 0;
            total_num = 1;
    }

    gamma(0) = -omega*r * std::sin(omega * t + (2* i * M_PI)/total_num); // I don't put i-1 as the article because there the i goes from 1 to 12, here from 0 to 11
    gamma(1) = -omega*r * std::cos(omega * t + (2* i * M_PI)/total_num); // Circular Formation
    return gamma;
}

Eigen::Vector4d articleConstantValues::getprofile(double t, int i)
{
    if(t<60){
        return calculateProfile(t,i, no_subgroup);
    }else{
        switch(i){
            case 0:
            case 1:
            case 2:
            case 3:
                return calculateProfile(t,i,sub_v1);
            break;
            case 4:
            case 5:
            case 6:
            case 7:
                return calculateProfile(t,i,sub_v2);
            break;
            case 8:
            case 9:
            case 10:
            case 11:
                return calculateProfile(t,i,sub_v3);
            break;
            default:
                return Eigen::Vector4d(0,0,0,0); // error
        }
    }
}

Eigen::Vector4d articleConstantValues::calculateProfile(double t, int i, Subgroups subgroup){ // Returns the profile function for that agent
    // The article uses a r = 20m and omega = 0.1 rad/s
    double r,omega;
    int total_num;
    Eigen::Vector4d profile(0,0,0,0);

    // Depending on the subgroup i can have different choices:
    switch(subgroup){
        case no_subgroup:
            r = 20;
            omega = 0.1;
            total_num = 12;
        break;
        case sub_v1:
            r = 20;
            omega = 0.1;
            total_num = 4;
        break;
        case sub_v2:
            r = 15;
            omega = 0.2;
            total_num = 4;
        break;
        case sub_v3:
            r = 10;
            omega = 0.3;
            total_num = 4;
        break;
        default: // Error
            omega = 0;
            r = 0;
            total_num = 1;
    }

    profile(0) = r * std::sin(omega * t + (2* i * M_PI)/total_num); // I don't put i-1 as the article because there the i goes from 1 to 12, here from 0 to 11
    profile(1) = r * omega * std::cos(omega * t + (2* i * M_PI)/total_num); // Circular Formation
    profile(2) = r * std::cos(omega * t + (2* i * M_PI)/12);
    profile(3) = -r * omega * std::sin(omega * t + (2* i * M_PI)/total_num); // Circular Formation
    return profile;
}
