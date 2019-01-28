//includes
#include <ros/ros.h>
#include <time.h>
#include <Eigen/Dense> 

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/AccelWithCovarianceStamped.h"
#include "nearlab_msgs/StateStamped.h"

#include "estimator.h"

// Global Variables
Estimator estimator;

//callbacks
//Each callback updates estimator
void imuCallback(const geometry_msgs::AccelWithCovarianceStamped msg){

}
void opticalFlowCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

}
void lidarFlowCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

}
// This comes from the RGB camera
void rgbPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

}
// This comes from the realsense camera
void lidarPoseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

}

int main(int argc, char** argv){
  ros::init(argc,argv,"sc_estimator");
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber subImu = nh.subscribe("/orbot/space/meas/imu",100,imuCallback);
  ros::Subscriber subOpticalFlow = nh.subscribe("/orbot/space/meas/optical_flow",100,opticalFlowCallback);
  ros::Subscriber subLidarFlow = nh.subscribe("/orbot/space/meas/lidar_flow",100,lidarFlowCallback);
  ros::Subscriber subRgbPose = nh.subscribe("/orbot/space/meas/rgb_pose",100,rgbPoseCallback);
  ros::Subscriber subLidarPose = nh.subscribe("/orbot/space/meas/lidar_pose",100,lidarPoseCallback);

  // Publishers
  pubState = nh.advertise<nearlab_msgs::StateStamped>("/orbot/space/state/estimate",100);
  //pubStateCov = nh.advertise<nearlab_msgs::StateWithCovarianceStamped>("/orbot/space/state/state_cov",1000);
  
  // Setup Estimator
  


  // Loop
  ros::Rate loop_rate(100);

  ROS_INFO("Estimator Node Listening");

  while(ros::ok()){
    // dynamics

    // publish

    nearlab::StateStamped stateMsg;
    state = estimator.getState();
    for(int i=0;i<3;i++){//There are better ways
      stateMsg.r[i] = state(i);
      stateMsg.q[i] = state(i+3);
      stateMsg.v[i] = state(i+7);
      stateMsg.ba[i] = state(i+10);
    }
    stateMsg.q[3] = state(6);
    ros::Time tsNow = ros::Time::now();
    stateMsg.tStamp = tsNow.toSec();
    pubState.publish(stateMsg);

    nearlab::StateCov stateCovMsg;
    P = estimator.getCovariance();
    for(int i=0;i<P.rows();i++){
      stateCovMsg.P[i] = P(i,i);
    }
    stateCovMsg.tStamp = tsNow.toSec();
    pubStateCov.publish(stateCovMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

