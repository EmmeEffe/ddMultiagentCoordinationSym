#include "MultiAgentControl.h"

MultiAgentControl::MultiAgentControl()
{
    articleValues = new articleConstantValues();
    c = std::vector<double>(articleValues->getM(), 0); // Initialize c to zero
}

// get the control for all the robots
std::vector<Eigen::Vector2d> MultiAgentControl::getControl(std::vector<Eigen::Vector4d> x, double time, std::vector<Eigen::Vector4d> &h, std::vector<Eigen::Vector4d> &errors){
    // u_i = (c_i + rho_i)*K*errors + gamma - mu*nonLinear(error)

    // Parameters
    /*
     * c > 0
     * sigma = Saturation value for u_k(t)
     * mu > sigma >= 0
    */

    std::string values;

    h.resize(articleValues->getM());

    // Get the profiles h for all the robots
    for(int i=0; i<articleValues->getM(); i++){
        h[i] = articleValues->getprofile(time, i);
    }

    errors.resize(articleValues->getM());

    articleValues->addTargetsToPositionList(x, time); // Add the targets to the position list

    // Check if any value in x is nan
    for(int i=0; i<(int)x.size(); i++){
        if(std::isnan(x[i].x()) || std::isnan(x[i].y()) || std::isnan(x[i].z()) || std::isnan(x[i].w())){
            throw std::runtime_error("Value in x["+std::to_string(i)+"] is NaN: " + std::to_string(x[i].x()) + ", " + std::to_string(x[i].y()) + ", " + std::to_string(x[i].z()) + ", " + std::to_string(x[i].w()));
        }
    }

    // Calculate all the errors
    errors = this->getErrors(x, h, time); // X has len N, h has len M

    // Check if any value in errors is nan
    for(int i=0; i<(int)errors.size(); i++){
        if(std::isnan(errors[i].x()) || std::isnan(errors[i].y()) || std::isnan(errors[i].z()) || std::isnan(errors[i].w())){
            throw std::runtime_error("Value in errors["+std::to_string(i)+"] is NaN: " + std::to_string(errors[i].x()) + ", " + std::to_string(errors[i].y()) + ", " + std::to_string(errors[i].z()) + ", " + std::to_string(errors[i].w()));
        }
    }

    // Integrate c_i_dot
    double old_c;
    for(int i=0; i<articleValues->getM(); i++){
        old_c = c[i];
        c[i] = c[i] + c_dot_i(errors[i]) * timeStep;
        if(std::isnan(c[i])){
            throw std::runtime_error("Value in c["+std::to_string(i)+"] is NaN: " + std::to_string(c[i]) + "Old c value is: " + 
            std::to_string(old_c) + ", c_dot_i is: " + std::to_string(c_dot_i(errors[i])) + "Errors is: " + std::to_string(errors[i].x()) +
             ", " + std::to_string(errors[i].y()) + ", " + std::to_string(errors[i].z()) + ", " + std::to_string(errors[i].w()));
        }
    }

    std::vector<Eigen::Vector2d> nonLinear = this->nonLinearFunctions(articleValues->getB(), articleValues->getP(), errors);

    // Check if any value in nonLinear is nan
    for(int i=0; i<(int)nonLinear.size(); i++){
        if(std::isnan(nonLinear[i].x()) || std::isnan(nonLinear[i].y())){
            throw std::runtime_error("Value in nonLinear["+std::to_string(i)+"] is NaN: " + std::to_string(nonLinear[i].x()) + ", " + std::to_string(nonLinear[i].y()));
        }
    }

    std::vector<Eigen::Vector2d> u(articleValues->getM());

    for(int i=0; i<articleValues->getM(); i++){
        u[i] = (c[i] + rho_i(errors[i])) * (articleValues->getK() * errors[i]) + articleValues->getgamma(time, i) - articleValues->getMu() * nonLinear[i];
    }

    return u;
}

std::vector<Eigen::Vector4d> MultiAgentControl::getErrors(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, double t) // Return error array
{
    std::vector<Eigen::Vector4d> errors(articleValues->getM());
    for(int i=0; i<articleValues->getM(); i++){ // For every element in vector
        errors[i] = getError(x, h, i, t);
    }
    return errors;
}

Eigen::Vector4d MultiAgentControl::getError(std::vector<Eigen::Vector4d> x, std::vector<Eigen::Vector4d> h, int i, double t){
    Eigen::Vector4d err(0, 0, 0, 0);
    Eigen::Vector4d thisErr = x[i] - h[i];
    if(std::isinf(thisErr.x()) || std::isinf(thisErr.y()) || std::isinf(thisErr.z()) || std::isinf(thisErr.w())){
        throw std::runtime_error("Value in thisErr is infinite for i = " + std::to_string(i) + ": " + std::to_string(thisErr.x()) + ", " + std::to_string(thisErr.y()) + ", " + std::to_string(thisErr.z()) + ", " + std::to_string(thisErr.w()));
    }
    for(int j=0; j<articleValues->getM(); j++){
        err = err + articleValues->getWeight(i,j, t) * (thisErr-(x[j] - h[j]));
        if(std::isinf(err.x()) || std::isinf(err.y()) || std::isinf(err.z()) || std::isinf(err.w())){
            throw std::runtime_error("Value in err is infinite for i = " + std::to_string(i) + ", j = " + std::to_string(j) + ": " + std::to_string(err.x()) + 
            ", " + std::to_string(err.y()) + ", " + std::to_string(err.z()) + ", " + std::to_string(err.w()) + ". Formula values: thisErr = " +
             std::to_string(thisErr.x()) + ", " + std::to_string(thisErr.y()) + ", " + std::to_string(thisErr.z()) + ", " + std::to_string(thisErr.w()) +
              ", x[j] = " + std::to_string(x[j].x()) + ", " + std::to_string(x[j].y()) + ", " + std::to_string(x[j].z()) + ", " + std::to_string(x[j].w()) +
               ", h[j] = " + std::to_string(h[j].x()) + ", " + std::to_string(h[j].y()) + ", " + std::to_string(h[j].z()) + ", " + std::to_string(h[j].w()) + 
              "Weight = " + std::to_string(articleValues->getWeight(i,j,t)) + ", t = " + std::to_string(t));
        }
    }

    for(int k=articleValues->getM(); k<articleValues->getN(); k++){
        err = err + articleValues->getWeight(i,k,t) * (thisErr-x[k]);
        if(std::isinf(err.x()) || std::isinf(err.y()) || std::isinf(err.z()) || std::isinf(err.w())){
            throw std::runtime_error("Value in err is infinite for i = " + std::to_string(i) + ", j = " + std::to_string(k) + ": " + std::to_string(err.x()) + 
            ", " + std::to_string(err.y()) + ", " + std::to_string(err.z()) + ", " + std::to_string(err.w()) + ". Formula values: thisErr = " +
             std::to_string(thisErr.x()) + ", " + std::to_string(thisErr.y()) + ", " + std::to_string(thisErr.z()) + ", " + std::to_string(thisErr.w()) +
              ", x[j] = " + std::to_string(x[k].x()) + ", " + std::to_string(x[k].y()) + ", " + std::to_string(x[k].z()) + ", " + std::to_string(x[k].w()) + 
              "Weight = " + std::to_string(articleValues->getWeight(i,k,t)) + ", t = " + std::to_string(t));
        }
    }//Value in err is infinite for i = 3, j = 12: -inf, -inf, -inf, inf. Formula values: thisErr = -20.000000, -0.000000, -0.000000, 2.000000, x[j] = 0.000000, 0.000000, 0.000000, 0.000000


    if(std::isinf(err.x()) || std::isinf(err.y()) || std::isinf(err.z()) || std::isinf(err.w())){
        throw std::runtime_error("Value in err is infinite for i = " + std::to_string(i) + ": " + std::to_string(err.x()) + ", " + std::to_string(err.y()) + ", " + std::to_string(err.z()) + ", " + std::to_string(err.w()));
    }

    return err;
}

std::vector<Eigen::Vector2d> MultiAgentControl::nonLinearFunctions(Eigen::MatrixXd B, Eigen::MatrixXd P, std::vector<Eigen::Vector4d> error) {
    // No anti-chattering
    std::vector<Eigen::Vector2d> nonLinear(articleValues->getM());
    for(int i=0; i<articleValues->getM(); i++){
        Eigen::Vector2d val = B.transpose() * P * error[i];
        double norm = val.norm();
        if(norm!=0)
            nonLinear[i] = val/norm;
        else
            nonLinear[i] = val;
    }
    return nonLinear;
}

double MultiAgentControl::rho_i(Eigen::Vector4d error)
{
    return error.transpose() * articleValues->getRho() * error;
}

double MultiAgentControl::c_dot_i(Eigen::Vector4d error)
{
    return error.transpose() * articleValues->getGamma() * error;
}
