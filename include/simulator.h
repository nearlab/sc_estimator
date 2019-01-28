#include <Eigen/Dense>

#ifndef SIMULATOR_H
#define SIMULATOR_H

const Eigen::VectorXd simulateOpticalFlow(const Eigen::VectorXd& x);
const Eigen::VectorXd simulateLidarFlow(const Eigen::VectorXd& x);
const Eigen::VectorXd simulateRgbPose(const Eigen::VectorXd& x);
const Eigen::VectorXd simulateLidarPose(const Eigen::VectorXd& x);

#endif