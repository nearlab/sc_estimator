#include <Eigen/Dense>

#ifndef SIMULATOR_H
#define SIMULATOR_H

Eigen::VectorXd simulateOpticalFlow(const Eigen::VectorXd& x);
Eigen::VectorXd simulateLidarFlow(const Eigen::VectorXd& x);
Eigen::VectorXd simulateRgbPose(const Eigen::VectorXd& x);
Eigen::VectorXd simulateLidarPose(const Eigen::VectorXd& x);
Eigen::VectorXd simulateLidarRaw(const Eigen::VectorXd& x, const Eigen::VectorXd& rI);

#endif