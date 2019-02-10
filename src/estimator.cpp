#include "estimator.h"
#include <math.h>

Estimator::Estimator(){
  this->state = Eigen::VectorXd::Zero(13);
  this->P = Eigen::MatrixXd::Identity(12,12);
  this->zImu = Eigen::VectorXd::Zero(6);
  this->QImu = Eigen::MatrixXd::Zero(12,12);
  this->dtImu = 0;
  this->tImu = 0;
  this->tOpticalFlow = 0;
  this->tLidarFlow = 0;
  this->tRgbPose = 0;
  this->tLidarPose = 0;
  this->tLidarRaw = 0;
  this->sim = Simulator::Simulator(); // This should be set through launch options
}
void Estimator::propagate(const double& dt){
  if(dtImu == 0){
    return;
  }
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

void Estimator::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::VectorXd& zHat,
                       const Eigen::MatrixXd H, const double dt){
  Eigen::MatrixXd Y = H*this->P*H.transpose() + R;
  Eigen::MatrixXd C = this->P*H.transpose();
  Eigen::MatrixXd K = C*Y.inverse();
  Eigen::VectorXd dz = (z - zHat);
  Eigen::VectorXd dx = K*dz;
  this->state.segment(0,3) += dx.segment(0,3);
  Eigen::VectorXd dq(4);
  dq.head(3) = dx.segment(3,3);
  dq.tail(1) << 1;//sqrt(1-pow((dt*dx.segment(3,3)/2).norm(),2));
  this->state.segment(3,4) = quatRot(this->state.segment(3,4),dq);
  this->state.segment(3,4) /= this->state.segment(3,4).norm();
  this->state.segment(7,6) += dx.segment(6,6);


  // REPLACE THIS WITH JOSEPH UPDATE
  this->P = (Eigen::MatrixXd::Identity(state.size()-1,state.size()-1)-K*H)*this->P*(Eigen::MatrixXd::Identity(state.size()-1,state.size()-1)-K*H).transpose()+K*R*K.transpose();
  this->P = .5*(this->P + (this->P).transpose());
}

void Estimator::updateOpticalFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& t){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,state.size()-1); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  double dt = t - tOpticalFlow;
  tOpticalFlow = t;
  // H << math here
  H << z3, z3, i3, z3,
       z3, z3, z3, i3;
  this->update(z,R,sim.simulateOpticalFlow(this->state),H,dt);
}

void Estimator::updateLidarFlow(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& t){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,state.size()-1); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  double dt = t - tLidarFlow;
  tLidarFlow = t;
  // H << math here
  H << z3, z3, i3, z3,
       z3, z3, z3, i3;
  this->update(z,R,sim.simulateLidarFlow(this->state),H,dt);
}

void Estimator::updateRgbPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& t){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,state.size()-1); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  double dt = t - tRgbPose;
  tRgbPose = t;
  // H << math here
  H << i3, z3, z3, z3,
       z3, i3, z3, z3;
  this->update(z.head(6),R,sim.simulateRgbPose(this->state),H,dt);
}

void Estimator::updateLidarPose(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& t){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7,state.size()-1); // dz/dx
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  double dt = t - tLidarPose;
  tLidarPose = t;
  // H << math here
  H << i3, z3, z3, z3,
       z3, i3, z3, z3;
  this->update(z.head(6),R,sim.simulateLidarPose(this->state),H,dt);
}

void updateLidarRaw(const Eigen::VectorXd& z, constEigen::MatrixXd& rI, const Eigen::MatrixXd R, const double& dt){
  Eigen::MatrixXd H = Eigen::MatrixXd::Zeros(3*n,state.size()-1);
  Eigen::Matrix3d z3 = Eigen::MatrixXd::Zero(3,3);
  Eigen::Matrix3d i3 = Eigen::MatrixXd::Identity(3,3);
  double dt = t - tLidarRaw;
  tLidarRaw = t;
  
  int n = rI.cols;
  Eigen::MatrixXd RBI = quat2rot(this->state.segment(3,4));
  Eigen::MatrixXd rB = this->state.head(3);
  Eigen::Matrix3d dzdqx,dzdqy,dzdqz;
  dzdqx <<  0    , 2*qy , 2*qz ,
            2*qy , -4*qx, -2*qw,
            2*qz , 2*qw , -4*qx;

  dzdqx <<  -4*qy, 2*qx , 2*qw ,
            2*qx , 0    , 2*qz ,
            -2*qw, 2*qz , -4*qy;

  dzdqx <<  -4*qz, -2*qw , 2*qx,
            2*qw , -4*qz, 2*qy ,
            2*qx , 2*qy , 0    ;

  for(int i=0;i<n;i++){
      Eigen::Matrix3d Hz = Eigen::MatrixXd::Zeros(3,3);
      Eigen::Matrix3d Hr = Eigen::MatrixXd::Zeros(3,6);
      Eigen::Vector3d rC = this->RCB * RBI * (rI.col(i) - rB) - this->RCB * this->rCam;
      double theta = atan2(rC(0),rC(2));
      double phi = atan2(cos(theta)*rC(1),rC(2));
      double rho = rC(1)/sin(theta);
      double dTdrx = (1/(1+pow(rC(0)/rC(2),2))*1/rC(2);
      double dTdry = 0;
      double dTdrz = dTdrz*(-rC(0)/rC(2));
      double dPdrx = (1/(1+pow(cos(theta)*(rC(1)/rC(2)),2)))*(-sin(theta)*rC(1)/rC(2)*dTdrz);
      double dPdry = cos(theta)*rC(2)/(rC(2)*rC(2)+pow(cos(theta)*rC(1),2));
      double dPdrz = (-cos(theta)*rC(1)/(rC(2)*rC(2)+pow(cos(theta)*rC(1),2)))*dPdrx;
      double dRdrx = -cos(phi)*rC(1)/sin(phi)/sin(phi)*dPdrx;
      double dRdry = 1/sin(phi)*(1 - rC(1)/tan(phi)*dPdrx);
      double dRdrz = -cos(phi)*rC(1)/sin(phi)/sin(phi)*dPdrx;
      
      Hz << dRdrx,dRdry,dRdrz,
            dPdrx,dPdry,dPdrz,
            dTdrx,dTdry,dTdrz;

      Hr.block(0,0,3,3) << -sim.RCB*RBI;
      Hr.col(3) << sim.RCB*dzdqx*(rI.col(i)-rB);
      Hr.col(4) << sim.RCB*dzdqy*(rI.col(i)-rB);
      Hr.col(5) << sim.RCB*dzdqz*(rI.col(i)-rB);

      H.block(3*i,0,3,6) << Hz*Hr;
  }
  this.update(z,R,sim.simulateLidarRaw(this->state),H,dt);
}

void Estimator::updateImu(const Eigen::VectorXd& z, const Eigen::MatrixXd R, const double& t){
  this->zImu = z;
  this->QImu = R;
  if(tImu > 0){
    this->dtImu = t - this->tImu;
  }
  this->tImu = t;
}

Eigen::VectorXd Estimator::getState(){
  return this->state;
}
Eigen::MatrixXd Estimator::getCovariance(){
  return this->P;
}
