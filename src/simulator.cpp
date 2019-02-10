#include "simulator.h"
#include <math.h>

Simulator::Simulator(){
    this->rCam = Eigen::VectorXd::Zeros(3);
    this->RCB = Eigen::MatrixXd::Identity(3,3);
}
Eigen::VectorXd Simulator::simulateOpticalFlow(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.tail(6);
    return zHat;

}
Eigen::VectorXd Simulator::simulateLidarFlow(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.tail(6);
    return zHat;
}
Eigen::VectorXd Simulator::simulateRgbPose(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.head(6);
    return zHat;
}
Eigen::VectorXd Simulator::simulateLidarPose(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.head(6);
    return zHat;
}

Eigen::VectorXd Simulator::simulateLidarRaw(const Eigen::VectorXd& x, const Eigen::MatrixXd& rI){    
    int n = rI.cols;
    Eigen::VectorXd zHat = Eigen::VectorXd::Zeros(3*n);
    Eigen::MatrixXd RBI = quat2rot(x.segment(3,4));
    Eigen::MatrixXd rB = x.segment(0,3);
    for(int i=0;i<n;i++){
        Eigen::Vector3d rC = this->RCB * RBI * (rI.col(i) - rB) - this->RCB * this->rCam;
        double theta = atan2(rC(0),rC(2));
        double phi = atan2(cos(theta)*rC(1),rC(2));
        double rho = rC(1)/sin(theta);
        zHat.segment(3*i,3) << rho,theta,phi;
    }
    return zHat;    
}