#include "estimator.h"
#include "simulator.h"

Estimator::Estimator(){
  this->state = Eigen::VectorXd::Zero(13);
  this->P = Eigen::MatrixXd::Identity(12,12);
  this->zImu = Eigen::VectorXd::Zero(6);
  this->QImu = Eigen::MatrixXd::Zero(12,12);
}
void Estimator::propagate(const double& dt){
  Eigen::VectorXd r(3),v(3),q(4),w(3);

  r = state.segment(0,3);
  v = state.segment(7,3);
  q = state.segment(3,4);
  w = state.segment(10,3);
  Eigen::Matrix3d T = quat2rot(q);
  Eigen::Matrix3d Tt = T.transpose();
  Eigen::Vector3d wk = zImu.tail(3);
  Eigen::VectorXd dq(4);
  dq.head(3) = dt*wk/2;
  dq.tail(1) << 1;//sqrt(1-pow((dt*wk/2).norm(),2));
  Eigen::Vector3d ak = zImu.head(3) - T*(Eigen::Vector3d() << 0,0,9.8).finished();
  Eigen::Matrix3d wkx,akx;
  wkx = crossProductEquivalent(wk);
  akx = crossProductEquivalent(ak);
  

  Eigen::VectorXd rk(3),vk(3),qk(4);
  rk = r + v*dt + 1/2*Tt*ak*pow(dt,2) - 1/6*Tt*akx*wk*pow(dt,3);
  vk = v + ak*dt - 1/2*Tt*akx*wk*pow(dt,2);
  qk = quatRot(q,dq);
  qk.head(2) << 0,0;
  qk = qk/qk.norm();

  state.segment(0,3) = rk;
  state.segment(7,3) = vk;
  state.segment(3,4) = qk;
  state.segment(10,3) = wk;

  //Update error covariance
  Eigen::Matrix3d wxax = crossProductEquivalent(wkx*ak);
  Eigen::MatrixXd F,M;
  F = Eigen::MatrixXd(12,12);
  M = Eigen::MatrixXd(12,9);
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  F << i3, pow(dt,3)/6*Tt*wxax-pow(dt,2)/2*Tt*akx, dt*i3, pow(dt,3)/6*Tt*wkx-pow(dt,2)/2*Tt, 
       z3, quat2rot(dq/dq.norm())                , z3   , z3,
       z3, pow(dt,2)/2*Tt*wxax-dt*Tt*akx         , i3   , pow(dt,2)*Tt*wkx-dt*Tt,
       z3, z3                                    , z3   , i3;

  this->P = F*this->P*F.transpose() + this->QImu;
}

void Estimator::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::VectorXd (*simulateMeas)(const Eigen::VectorXd& x),
                       const Eigen::MatrixXd H, const double dt){
  Eigen::MatrixXd Y = H*this->P*H.transpose() + R;
  Eigen::MatrixXd C = this->P*H.transpose();
  Eigen::MatrixXd K = C*Y.inverse();
  Eigen::VectorXd dz = (z - simulateMeas(this->state));
  Eigen::VectorXd dx = K*dz;
  this->state.segment(0,3) += dx.segment(0,3);
  Eigen::VectorXd dq(4);
  dq.head(3) = dx.segment(3,3);
  dq.tail(1) << 1;//sqrt(1-pow((dt*dx.segment(3,3)/2).norm(),2));
  this->state.segment(3,4) = quatRot(this->state.segment(3,4),dq);
  this->state.segment(3,4) /= this->state.segment(3,4).norm();
  this->state.segment(7,6) += dx.segment(6,6);


  // REPLACE THIS WITH JOSEPH UPDATE
  this->P = (Eigen::MatrixXd::Identity(12,12)-K*H)*this->P*(Eigen::MatrixXd::Identity(12,12)-K*H).transpose()+K*R*K.transpose();
  this->P = .5*(this->P + (this->P).transpose());
}

void Estimator::updateOpticalFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,12); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  // H << math here
  this->update(z,R,simulateOpticalFlow,H,dt);
}

void Estimator::updateLidarFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,12); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  // H << math here
  this->update(z,R,simulateLidarFlow,H,dt);
}

void Estimator::updateRgbPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,12); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  // H << math here
  this->update(z,R,simulateRgbPose,H,dt);
}

void Estimator::updateLidarPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,12); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  // H << math here
  this->update(z,R,simulateLidarPose,H,dt);
}

void Estimator::updateImu(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& dt){
  this->zImu = z;
  this->QImu = R;
}

Eigen::VectorXd Estimator::getState(){
  return this->state;
}
Eigen::MatrixXd Estimator::getCovariance(){
  return this->P;
}
