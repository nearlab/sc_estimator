#include "simulator.h"

Eigen::VectorXd simulateOpticalFlow(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.tail(6);
    return zHat;

}
Eigen::VectorXd simulateLidarFlow(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.tail(6);
    return zHat;
}
Eigen::VectorXd simulateRgbPose(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.head(6);
    return zHat;
}
Eigen::VectorXd simulateLidarPose(const Eigen::VectorXd& x){
    Eigen::VectorXd zHat = x.head(6);
    return zHat;
}