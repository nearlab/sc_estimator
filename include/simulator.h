#include <Eigen/Dense>

#ifndef SIMULATOR_H
#define SIMULATOR_H
class Simulator{
public: 
  Eigen::Vector3d rCam;
  Eigen::Matrix3d RCB;
  Simulator();
  Simulator(Eigen::Vector3d rCam, Eigen::Matrix3d RCB):rCam(rCam),RCB(RCB){}
  Eigen::VectorXd simulateOpticalFlow(const Eigen::VectorXd& x);
  Eigen::VectorXd simulateLidarFlow(const Eigen::VectorXd& x);
  Eigen::VectorXd simulateRgbPose(const Eigen::VectorXd& x);
  Eigen::VectorXd simulateLidarPose(const Eigen::VectorXd& x);
  Eigen::VectorXd simulateLidarRaw(const Eigen::VectorXd& x, const Eigen::MatrixXd& rI);
};
#endif