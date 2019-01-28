#include "simulator.h"

const Eigen::VectorXd simulateOpticalFlow(const Eigen::VectorXd& x){
    return Eigen::VectorXd::Zero(6);
}
const Eigen::VectorXd simulateLidarFlow(const Eigen::VectorXd& x){
    return Eigen::VectorXd::Zero(6);
}
const Eigen::VectorXd simulateRgbPose(const Eigen::VectorXd& x){
    return Eigen::VectorXd::Zero(7);
}
const Eigen::VectorXd simulateLidarPose(const Eigen::VectorXd& x){
    return Eigen::VectorXd::Zero(7);
}