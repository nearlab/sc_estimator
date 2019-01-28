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
  ros::Publisher pubState = nh.advertise<nearlab_msgs::StateStamped>("/orbot/space/state/estimate",100);
  //pubStateCov = nh.advertise<nearlab_msgs::StateWithCovarianceStamped>("/orbot/space/state/state_cov",1000);
  
  // Setup Estimator
  


  // Loop
  ros::Rate loop_rate(100);
  ros::Time tPrev = ros::Time::now();
  ROS_INFO("Estimator Node Listening");
  int sequence = 0;

  while(ros::ok()){
    // dynamics
    ros::Time tCurr = ros::Time::now();
    double dt = (tCurr - tPrev).toSec();
    tPrev = tCurr;
    estimator.propagate(dt);

    // publish

    nearlab_msgs::StateStamped stateMsg;
    Eigen::VectorXd state = estimator.getState();
    stateMsg.header.seq = sequence++;
    stateMsg.header.stamp = ros::Time::now();
    stateMsg.r.x = state(0);
    stateMsg.r.y = state(1);
    stateMsg.r.z = state(2);
    stateMsg.q.x = state(3);
    stateMsg.q.y = state(4);
    stateMsg.q.z = state(5);
    stateMsg.q.w = state(6);
    stateMsg.v.x = state(7);
    stateMsg.v.y = state(8);
    stateMsg.v.z = state(9);
    stateMsg.w.x = state(10);
    stateMsg.w.y = state(11);
    stateMsg.w.z = state(12);
    pubState.publish(stateMsg);

    // nearlab::StateCov stateCovMsg;
    // P = estimator.getCovariance();
    // for(int i=0;i<P.rows();i++){
    //   stateCovMsg.P[i] = P(i,i);
    // }
    // stateCovMsg.tStamp = tsNow.toSec();
    // pubStateCov.publish(stateCovMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

