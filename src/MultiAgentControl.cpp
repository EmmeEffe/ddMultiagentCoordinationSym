#include "MultiAgentControl.h"

#include <fstream>

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

    h.resize(articleValues->getN());

    // Get the profiles h for all the robots
    for(int i=0; i<articleValues->getM(); i++){
        h[i] = articleValues->getprofile(time, i);
    }
    for(int i=articleValues->getM(); i<articleValues->getN(); i++){
        h[i] = articleValues->getTargetPosition(i, time);
    }

    errors.resize(articleValues->getM());

    articleValues->addTargetsToPositionList(x, time); // Add the targets to the position list

    // Calculate all the errors
    errors = this->getErrors(x, h, time); // X has len N, h has len M

    // Integrate c_i_dot
    for(int i=0; i<articleValues->getM(); i++){
        c[i] = c[i] + c_dot_i(errors[i]) * timeStep;
    }

    std::vector<Eigen::Vector2d> nonLinear = this->nonLinearFunctions(articleValues->getB(), articleValues->getP(), errors);

    std::vector<Eigen::Vector2d> u(articleValues->getM());

    for(int i=0; i<articleValues->getM(); i++){
        u[i] = (c[i] + rho_i(errors[i])) * (articleValues->getK() * errors[i]) + articleValues->getgamma(time, i) - articleValues->getMu() * nonLinear[i];
        //u[i] = 10 * (articleValues->getK() * errors[i]) + articleValues->getgamma(time, i) - articleValues->getMu() * nonLinear[i];
    }

    #ifdef DEBUG
    // File Log Values for Debug Purposes
    std::ofstream myfile;
    myfile.open ("/home/martino/Desktop/log.txt", std::ios_base::app);
    myfile << "\n---\nTime: " << time << "\n";
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "Err[" << i << "] (" << errors[i](0) << ", " << errors[i](1) << ", "<< errors[i](2) << ", "<< errors[i](3) << ")\n";
    }
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "c_dot[" << i << "] (" << c_dot_i(errors[i]) <<"\n";
    }
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "c[" << i << "] (" << c[i]<<"\n";
    }
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "rho[" << i << "] (" <<  rho_i(errors[i]) <<"\n";
    }
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "nonLinear[" << i << "] (" << nonLinear[i](0) << ", " << nonLinear[i](1) << ")\n";
    }
    for(int i=0; i<articleValues->getM(); i++){
        myfile << "u[" << i << "] (" << u[i](0) << ", " << u[i](1) << ")\n";
    }
    #endif

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
    for(int j=0; j<articleValues->getM(); j++){
        err = err + articleValues->getWeight(i,j, t) * (thisErr-(x[j] - h[j]));
    }

    for(int k=articleValues->getM(); k<articleValues->getN(); k++){
        err = err + articleValues->getWeight(i,k,t) * (thisErr-x[k]);
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
