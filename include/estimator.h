#include <Eigen/Dense>
#include <math.h>
#include "quatMath.h"

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

class Estimator{
public:
  Estimator();
  void propagate(const double& dt);
  void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, Eigen::VectorXd (*simulateMeas)(const Eigen::VectorXd& x),
                         const Eigen::MatrixXd H, const double dt);
  void updateOpticalFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt);
  void updateLidarFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt);
  void updateRgbPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt);
  void updateLidarPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt);
  void updateLidarRaw(const Eigen::VectorXd& z, constEigen::VectorXd& rI, const Eigen::MatrixXd R, const double& dt);
  void updateImu(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt);
  Eigen::VectorXd getState();
  Eigen::MatrixXd getCovariance();
private:
  Eigen::VectorXd state;  
  Eigen::MatrixXd P;
  Eigen::VectorXd zImu;
  Eigen::MatrixXd QImu;
  double dtImu, tImu, tOpticalFlow, tLidarFlow, tRgbPose, tLidarPose;
  int updateCount;
};
#endif
