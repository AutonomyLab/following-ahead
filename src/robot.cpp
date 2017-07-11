#include "filter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include "robot.hpp"
#include "config.h"
#include "pid.h"
#include "kalman.hpp"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "utils.hpp"

cv::Point3f Robot::getRobotPose()
{
	return robot_poses->getFilter();
}	

cv::Point3f Robot::getHumanPose()
{
	return human_poses->getFilter();
}

Robot::Robot(  ros::NodeHandle n)
{
  cv::Mat Q = cv::Mat::zeros(NUM_STATES, NUM_STATES, CV_32F);
  Q.at<float>(0, 0) = Q0;
  Q.at<float>(1, 1) = Q1;
  Q.at<float>(2, 2) = Q2;
  Q.at<float>(3, 3) = Q3;
  Q.at<float>(4, 4) = Q4;
  Q.at<float>(5, 5) = Q5;
  Q.at<float>(6, 6) = Q6;
  Q.at<float>(7, 7) = Q7;
  Q.at<float>(8, 8) = Q8;

  cv::Mat R = cv::Mat::zeros(4, 4, CV_32F);
  R.at<float>(0, 0) = R0;
  R.at<float>(1, 1) = R1;
  R.at<float>(2, 2) = R2;
  R.at<float>(3, 3) = R3;

  cv::Mat P = cv::Mat::eye(NUM_STATES, NUM_STATES, CV_32F);
  P.at<float>(0, 0) = P0;
  P.at<float>(1, 1) = P1;
  P.at<float>(2, 2) = P2;
  P.at<float>(3, 3) = P3;
  P.at<float>(4, 4) = P4;
  P.at<float>(5, 5) = P5;
  P.at<float>(6, 6) = P6;
  P.at<float>(7, 7) = P7;
  P.at<float>(8, 8) = P8;
  P.at<float>(9, 9) = P6;
  P.at<float>(10, 10) = P7;

  // pub_waypoints_ = nh_.advertise<sensor_msgs::PointCloud>("/person_follower/waypoints", 1);

  kalman_filter = new KalmanFilter(0.1, Q, R, P);
  cmd_vel_publisher =  n.advertise<geometry_msgs::Twist>("/husky/cmd_vel", 1);
  isDeadManActive = false;
	robot_poses = new Filter(ROBOT_FILTER_SIZE);
	human_poses = new Filter(PERSON_FILTER_SIZE);
	destination_pose = new Filter(DESTINATION_FILTER_SIZE);
	human_prev_pose = cv::Point3f(0,0,0);
  human_relative_pose = cv::Point3f(0,0,0);
  human_prev_vel = cv::Point3f(0,0,0);
  robot_prev_cmd_vel = cv::Point3f(0,0,0);

  robot_prev_pose = cv::Point3f(0,0,0);
  robot_prev_degree_dest = 0.0;
  referenceLastUpdated = 0;
	pid_turn.set(AVOID_TURN, -AVOID_TURN, AVOID_TURN, 0.0100, 0.01);
  pid_cruse.set(CRUISE_SPEED, -CRUISE_SPEED, -CRUISE_SPEED  , .0100, 0.001);

  if (tf_listener.waitForTransform( 
      "robot_0/base_footprint",
      "robot_0/odom",
      ros::Time(0),
      ros::Duration( 2 )
    )
  )
  {
    try
    {
      
      tf_listener.lookupTransform("robot_0/odom", "robot_0/base_footprint",
                                  ros::Time(0), local_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
    }
  }
  else
  {
    ROS_ERROR("Initial transform not found!!!");
    exit(0);
  }
}

void Robot::joyCallback(const sensor_msgs::Joy& msg)
{
  if (msg.buttons[0]==1 && msg.buttons[6]==1 && msg.buttons[7]==1 ) {
    isDeadManActive = true;
  }
  else 
    isDeadManActive = false;
  std::cout << "isDeadManActive: " << isDeadManActive << std::endl; 
}

void Robot::myBlobUpdate (const geometry_msgs::TransformStamped& msg)
{

  // tf::Matrix3x3 m(
  //   tf::Quaternion(
  //     msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
  //   )
  // );
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  human_relative_pose = cv::Point3f(msg.transform.translation.x, msg.transform.translation.y, 0);
  double speedX = 0;
  double speedY = 0;

  // maxblobx = blob->scan_width; //?

  
}

//-------------------------avg_dest is return parameter of this function ------------
int Robot::calculateDistination(cv::Point3f& avg_dest)
{
	//-----------------------adding new point to human pose history--------------------
	human_poses->addPoint(human_relative_pose);
  // std::cout << "human pose: "<< human_relative_pose << std::endl;

  //-----------------------gitting pos history average-------------------------------
  cv::Point3f human_avg_pose = cv::Point3f(kalman_filter->state().at<float>(5,0),kalman_filter->state().at<float>(6,0),0); 

  double speedX = kalman_filter->state().at<float>(7,0)*.1; 
  double speedY = kalman_filter->state().at<float>(8,0)*.1; 

  speedX = ( fabs(speedX) < VELOCITY_THRESHOLD ) ? 0 : speedX;
  speedY = ( fabs(speedY) < VELOCITY_THRESHOLD ) ? 0 : speedY;
      
  if (speedX == 0 && speedY == 0)
  {
    speedX = human_prev_vel.x;
    speedY = human_prev_vel.y;
  }
  human_prev_pose = cv::Point3f(human_avg_pose.x,human_avg_pose.y,0);
  human_prev_vel = cv::Point3f(speedX,speedY,0);
  //----------------avoid deviding by 0----------------------------------------------
  if (speedX*speedX + speedY*speedY == 0){   
    return 1;
  }
  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
  destination_pose->addPoint(cv::Point3f(human_avg_pose.x+speedX*2 + speedXU*FOLLOW_DIST
  													, human_avg_pose.y+speedY*2 + speedYU*FOLLOW_DIST
  													,0
  													));
  avg_dest = destination_pose->getFilter();
  
  return 0;
}


int Robot::publishCmdVel(cv::Point3f destination)
{
	cv::Point3f robot_pose = robot_poses->getFilter();
  if ((destination.x - robot_pose.x)==0) // Avoid devide by 0
    return 0;
  double m = (destination.y - robot_pose.y) / (destination.x - robot_pose.x);
  double degreeD = atan(m);

  if (destination.x < robot_pose.x && destination.y > robot_pose.y  )
    degreeD = M_PI + degreeD;  
  else if (destination.x < robot_pose.x && destination.y < robot_pose.y  )
    degreeD = -M_PI + degreeD;

  if (abs(degreeD -robot_prev_degree_dest) >= 180){
    // std::cout << "degreed + .: " << degreeD -robot_prev_degree_dest <<std::endl;
    degreeD = robot_prev_degree_dest;
  }
  robot_prev_degree_dest = degreeD;
  // std::cout << "m: "<< m << "atan: " <<  degreeD*180 / M_PI << std::endl;
  
  double Dturn = robot_pose.z - degreeD;

  if (Dturn >= M_PI)
    Dturn -= M_PI*2;
  else if (Dturn <= -M_PI)
     Dturn += M_PI*2;
  // std::cout << "dturn: " << Dturn << std::endl;
  //set turn speed
  if (!isDeadManActive)
    return 0;
  double Tspeed = pid_turn.calculate( 0 ,  Dturn , 0.01 );
  // std::cout << "Tspeed: " << Tspeed << std::endl;

  double distance = sqrt(pow(destination.x- robot_pose.x, 2) +  pow(destination.y - robot_pose.y, 2));
  double Xspeed = pid_cruse.calculate(0 ,distance, 0.01 );

  geometry_msgs::Twist cmd_vel_msg; 
  cmd_vel_msg.linear.x = Xspeed;
  cmd_vel_msg.angular.z = Tspeed; // Tspeed;

  
  robot_prev_cmd_vel.x = Xspeed;
  robot_prev_cmd_vel.z = Tspeed;

  cmd_vel_publisher.publish(cmd_vel_msg);
  return 0; 
}



void Robot::odometryCallback(const nav_msgs::Odometry& msg)
{
  cv::Point3f robot_pose;
  cv::Mat state = kalman_filter->state();
    
  if (referenceLastUpdated >= NUM_UPDATE_REF_FRAME)
  {
    robot_pose = robot_poses->getFilter();
    cv::Mat oldState = kalman_filter->state();
    
    float oldVelOther_x = oldState.at<float>(7, 0);
    float oldVelOther_y = oldState.at<float>(8, 0);

    cv::Mat rnew_T_r0 = xytheta2TransformationMatrix(robot_pose).inv();

    cv::Mat r0_v(2, 1, CV_32F);
    r0_v.at<float>(0, 0) = oldVelOther_x;
    r0_v.at<float>(1, 0) = oldVelOther_y;
    cv::Mat rnew_v = rnew_T_r0.rowRange(0, 2).colRange(0, 2) * r0_v;

    cv::Mat r0_previous_human_position = cv::Mat::zeros(3, 1, CV_32F);
    r0_previous_human_position.at<float>(0, 0) = oldState.at<float>(9, 0);
    r0_previous_human_position.at<float>(1, 0) = oldState.at<float>(10, 0);
    cv::Mat rnew_previous_human_position = rnew_T_r0 * r0_previous_human_position;


    robot_poses->addPoint(cv::Point3f(0, 0, 0));
    robot_prev_pose = cv::Point3f(0, 0, 0);
    referenceLastUpdated = 0;
    robot_pose = robot_prev_pose;


    cv::Mat newState = cv::Mat::zeros(11, 1, CV_32F);
    newState.at<float>(3, 0) = oldState.at<float>(3, 0);
    newState.at<float>(4, 0) = oldState.at<float>(4, 0);
    newState.at<float>(5, 0) = human_relative_pose.x;
    newState.at<float>(6, 0) = human_relative_pose.y;
    newState.at<float>(7, 0) = rnew_v.at<float>(0, 0);
    newState.at<float>(8, 0) = rnew_v.at<float>(1, 0);
    newState.at<float>(9, 0) = rnew_previous_human_position.at<float>(0, 0);
    newState.at<float>(10, 0) = rnew_previous_human_position.at<float>(1, 0);
  
    kalman_filter->init(0, newState);

    state = kalman_filter->state();

    // read the robot global position here as that is going to be our origin
    try
    {
      tf_listener.lookupTransform("robot_0/odom", "robot_0/base_footprint",  
                                  ros::Time(0), local_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      exit(0);
      // ros::Duration(1.0).sleep();
    }
  }
  else
  {
    double v_odom = msg.twist.twist.linear.x;  
    double omega_odom = msg.twist.twist.angular.z ;  

    double omega_cmd_vel = robot_prev_cmd_vel.z;
    double v_cmd_vel = robot_prev_cmd_vel.x;

    // measurement
    cv::Mat y = cv::Mat(4, 1, CV_32F);
    y.at<float>(0, 0) = v_odom;
    y.at<float>(1, 0) = omega_odom;
    y.at<float>(2, 0) = human_relative_pose.x;
    y.at<float>(3, 0) = human_relative_pose.y;

    // control input
    cv::Mat u = cv::Mat(2, 1, CV_32F);
    u.at<float>(0, 0) = v_cmd_vel;
    u.at<float>(1, 0) = omega_cmd_vel;

    if (!kalman_filter->isInitialized())
    {
      cv::Mat initState = cv::Mat::zeros(NUM_STATES, 1, CV_32F);
      initState.at<float>(5, 0) = human_poses->getFilter().x;
      initState.at<float>(6, 0) = human_poses->getFilter().y;
      initState.at<float>(9, 0) = human_poses->getFilter().x;
      initState.at<float>(10, 0) = human_poses->getFilter().y;

      kalman_filter->init(0, initState);
    }
    
    kalman_filter->update(y, 0.1, u);

    robot_pose.x = state.at<float>(0, 0);
    robot_pose.y = state.at<float>(1, 0);
    robot_pose.z = state.at<float>(2, 0);
    robot_pose.z = atan2(sin(robot_pose.z), cos(robot_pose.z));
    robot_poses->addPoint(robot_pose);
    
  }

  // broadcast the local frame with respect to the global frame
  local_transform.child_frame_id_ = "local_frame"; // source
  local_transform.frame_id_ = "robot_0/odom"; // target
  local_transform.stamp_ = ros::Time::now();
  tf_broadcaster.sendTransform(local_transform);

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(robot_pose.x, robot_pose.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, robot_pose.z);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "robot"));

  // ROS_INFO("Robot Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.z);
  cv::Point3f avg_dest = destination_pose->getFilter();

  transform.setOrigin( tf::Vector3(avg_dest.x, avg_dest.y, 0.0) );
  q.setRPY(0, 0, avg_dest.z);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "destination"));

  transform.setOrigin( tf::Vector3(state.at<float>(5, 0), state.at<float>(6, 0), 0.0) );
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "person"));

  cv::Point3f avgDestinations;
  if (calculateDistination(avgDestinations))
  {
    ROS_ERROR("calculation error");
    return;
  }

  publishCmdVel(avgDestinations);

  
  referenceLastUpdated++;
}

