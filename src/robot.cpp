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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "utils.hpp"

cv::Point3f Robot::getRobotPose()
{
	return robot_poses->getFilter();
}	

cv::Point3f Robot::getHumanPose()
{
	return human_poses->getFilter();
}

Robot::Robot(ros::NodeHandle n)
{
  cv::Mat Q = cv::Mat::zeros(NUM_STATES, NUM_STATES, CV_32F);
  Q.at<float>(X_T_IDX, X_T_IDX) = X_T_PROCESS_NOISE_VAR;
  Q.at<float>(Y_T_IDX, Y_T_IDX) = Y_T_PROCESS_NOISE_VAR;
  Q.at<float>(X_T_1_IDX, X_T_1_IDX) = X_T_1_PROCESS_NOISE_VAR;
  Q.at<float>(Y_T_1_IDX, Y_T_1_IDX) = Y_T_1_PROCESS_NOISE_VAR;
  Q.at<float>(VEL_IDX, VEL_IDX) = VEL_PROCESS_NOISE_VAR;
  Q.at<float>(THETA_IDX, THETA_IDX) = THETA_PROCESS_NOISE_VAR;

  cv::Mat R = cv::Mat::zeros(2, 2, CV_32F);
  R.at<float>(0, 0) = X_T_MEASUREMENT_NOISE_VAR;
  R.at<float>(1, 1) = Y_T_MEASUREMENT_NOISE_VAR;

  cv::Mat P = cv::Mat::eye(NUM_STATES, NUM_STATES, CV_32F);
  P.at<float>(0, 0) = X_T_INIT_ERROR_VAR;
  P.at<float>(1, 1) = Y_T_INIT_ERROR_VAR;
  P.at<float>(2, 2) = X_T_1_INIT_ERROR_VAR;
  P.at<float>(3, 3) = Y_T_1_INIT_ERROR_VAR;
  P.at<float>(4, 4) = VEL_INIT_ERROR_VAR;
  P.at<float>(5, 5) = THETA_INIT_ERROR_VAR;

  // pub_waypoints_ = nh_.advertise<sensor_msgs::PointCloud>("/person_follower/waypoints", 1);

  person_kalman_ = new PersonKalman(0.1, Q, R, P);

  cmd_vel_publisher =  n.advertise<geometry_msgs::Twist>("/husky/cmd_vel", 1);
  pub_particles_ = n.advertise<nav_msgs::GridCells>("person_follower/person_particle", 1);
  pub_nav_goal_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal_unthrottled", 1);

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

  tf::StampedTransform r0_T_map; 
  try
  {
    
    tf_listener.lookupTransform("robot_0/base_footprint", "map",
                                ros::Time(0), r0_T_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error %s", ex.what());
    return;
    // ros::Duration(1.0).sleep();
  }

  tf::Transform transform_r0_r1;
  transform_r0_r1.setOrigin( tf::Vector3(msg.transform.translation.x, msg.transform.translation.y, 0.0) );
  transform_r0_r1.setRotation(r0_T_map.getRotation());
  tf_broadcaster.sendTransform(tf::StampedTransform(transform_r0_r1, ros::Time::now(), "robot_0/base_footprint", "robot_1/base_footprint"));

  ROS_INFO("Broadcasting robot_0, robot_1 tf");

  if (!particle_filter_.isInitialized())
  {
    particle_filter_.init(
      NUM_PARTICLES, BLOB_NOISE_STDDEV_X, BLOB_NOISE_STDDEV_Y, PARTICLE_STOCHASTIC_VELOCITY_STDDEV,
      human_relative_pose, r0_T_map 
    );
    ROS_INFO("particle_filter_ initialized!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  else
  {
    // update the damn particles here based on measurement
    particle_filter_.update(human_relative_pose, r0_T_map);
  }

  // lets publish particles
  nav_msgs::GridCells particles;
  particles.header.stamp = ros::Time::now();
  particles.header.frame_id = "map";
  particles.cell_width = 0.1  ;
  particles.cell_height = 0.1;
  
  ROS_INFO("Num particles: %d", particle_filter_.getNumParticles());
  for (size_t i = 0, len = particle_filter_.getNumParticles(); i < len; i++)
  {
    ParticleFilter::Particle particle = particle_filter_.getParticleAt(i);
    cv::Point3f particle_state = particle.getState();

    geometry_msgs::Point point;
    point.x = particle_state.x;
    point.y = particle_state.y;
    point.z = 0;
    particles.cells.push_back(point);

    if (fabs(point.x) < 0.001 && fabs(point.y) < 0.001)
    {
      ROS_ERROR("Particle %d at origin", i);
    }
  }
  
  pub_particles_.publish(particles);

  
  






  cv::Point3f robot_pose;
  cv::Mat state = person_kalman_->state();
  
  // global pose
  cv::Point3f human_global_pose = transformPoint(r0_T_map.inverse(), human_relative_pose);
  // measurement
  cv::Mat y = cv::Mat(2, 1, CV_32F);
  y.at<float>(0, 0) = human_global_pose.x;
  y.at<float>(1, 0) = human_global_pose.y;

  if (!person_kalman_->isInitialized())
  {
    cv::Mat initState = cv::Mat::zeros(NUM_STATES, 1, CV_32F);
    initState.at<float>(X_T_IDX, 0) = human_global_pose.x;
    initState.at<float>(Y_T_IDX, 0) = human_global_pose.y;
    initState.at<float>(X_T_1_IDX, 0) = human_global_pose.x;
    initState.at<float>(Y_T_1_IDX, 0) = human_global_pose.y;
    initState.at<float>(VEL_IDX, 0) = 0;
    initState.at<float>(THETA_IDX, 0) = 0;

    person_kalman_->init(0, initState);
  }
  
  person_kalman_->update(y, 0.1);
  cv::Mat new_state = person_kalman_->state();

  // TODO: MAKE THE TFs ROS PARAM!!!
  // broadcast the estimated person position frame with respect to the global frame
  tf::StampedTransform person_kalman_transform;
  person_kalman_transform.child_frame_id_ = "person_kalman"; // source
  person_kalman_transform.frame_id_ = "map"; // target
  person_kalman_transform.stamp_ = ros::Time::now();
  
  float est_x = new_state.at<float>(X_T_IDX, 0);
  float est_y = new_state.at<float>(Y_T_IDX, 0);
  float est_v = new_state.at<float>(VEL_IDX, 0);
  float est_theta = new_state.at<float>(THETA_IDX, 0);

  // float est_x = human_global_pose.x;
  // float est_y = human_global_pose.y;
  // float est_theta = 0;

  if (!std::isnan(est_x) && !std::isnan(est_y))
  {
    person_kalman_transform.setOrigin( 
      tf::Vector3(
        est_x,
        est_y,
        0
      ) 
    );
  }
  else
  {
    ROS_ERROR("Kalman filter returned NaN position");
    return;
  }

  if (!std::isnan(est_theta))
  {  
    tf::Quaternion q;
    q.setRPY(0, 0, est_theta);
    person_kalman_transform.setRotation(q);
  }
  else
  {
    person_kalman_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    ROS_ERROR("Kalman filter returned NaN orientation");
    return;
  }

  tf_broadcaster.sendTransform(person_kalman_transform);

  // ------------------------ Predict position ------------------------ //
  tf::StampedTransform prediction_kalman_transform;
  prediction_kalman_transform.child_frame_id_ = "prediction_kalman"; // source
  prediction_kalman_transform.frame_id_ = "map"; // target
  prediction_kalman_transform.stamp_ = ros::Time::now();

  float prediction_x = est_x + cos(est_theta) * PREDICTION_LOOKAHEAD_DISTANCE;
  float prediction_y = est_y + sin(est_theta) * PREDICTION_LOOKAHEAD_DISTANCE;

  prediction_kalman_transform.setOrigin(tf::Vector3(prediction_x, prediction_y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, est_theta);
  prediction_kalman_transform.setRotation(q);

  tf_broadcaster.sendTransform(prediction_kalman_transform);

  // tell the robot to go to the predicted position
  if (est_v > DISTANCE_EPSILON/10)
  {
    geometry_msgs::PoseStamped nav_goal_msg;
    nav_goal_msg.header.stamp = ros::Time::now();
    nav_goal_msg.header.frame_id = "map";
    nav_goal_msg.pose.position.x = prediction_x;
    nav_goal_msg.pose.position.y = prediction_y;
    nav_goal_msg.pose.position.z = 0;
    nav_goal_msg.pose.orientation.x = q.x();
    nav_goal_msg.pose.orientation.y = q.y();
    nav_goal_msg.pose.orientation.z = q.z();
    nav_goal_msg.pose.orientation.w = q.w();
    pub_nav_goal_.publish(nav_goal_msg);
  }

  // // ROS_INFO("Robot Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.z);
  // cv::Point3f avg_dest = destination_pose->getFilter();

  // transform.setOrigin( tf::Vector3(avg_dest.x, avg_dest.y, 0.0) );
  // q.setRPY(0, 0, avg_dest.z);
  // transform.setRotation(q);
  // tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "destination"));

  // transform.setOrigin( tf::Vector3(state.at<float>(5, 0), state.at<float>(6, 0), 0.0) );
  // q.setRPY(0, 0, 0);
  // transform.setRotation(q);
  // tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "person"));

  // cv::Point3f avgDestinations;
  // if (calculateDistination(avgDestinations))
  // {
  //   ROS_ERROR("calculation error");
  //   return;
  // }

  // publishCmdVel(avgDestinations);

}

//-------------------------avg_dest is return parameter of this function ------------
int Robot::calculateDistination(cv::Point3f& avg_dest)
{
	//-----------------------adding new point to human pose history--------------------
	// human_poses->addPoint(human_relative_pose);
 //  // std::cout << "human pose: "<< human_relative_pose << std::endl;

 //  //-----------------------gitting pos history average-------------------------------
 //  cv::Point3f human_avg_pose = cv::Point3f(kalman_filter->state().at<float>(5,0),kalman_filter->state().at<float>(6,0),0); 

 //  double speedX = kalman_filter->state().at<float>(7,0)*.1; 
 //  double speedY = kalman_filter->state().at<float>(8,0)*.1; 

 //  speedX = ( fabs(speedX) < VELOCITY_THRESHOLD ) ? 0 : speedX;
 //  speedY = ( fabs(speedY) < VELOCITY_THRESHOLD ) ? 0 : speedY;
      
 //  if (speedX == 0 && speedY == 0)
 //  {
 //    speedX = human_prev_vel.x;
 //    speedY = human_prev_vel.y;
 //  }
 //  human_prev_pose = cv::Point3f(human_avg_pose.x,human_avg_pose.y,0);
 //  human_prev_vel = cv::Point3f(speedX,speedY,0);
 //  //----------------avoid deviding by 0----------------------------------------------
 //  if (speedX*speedX + speedY*speedY == 0){   
 //    return 1;
 //  }
 //  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
 //  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
 //  destination_pose->addPoint(cv::Point3f(human_avg_pose.x+speedX*2 + speedXU*FOLLOW_DIST
 //  													, human_avg_pose.y+speedY*2 + speedYU*FOLLOW_DIST
 //  													,0
 //  													));
 //  avg_dest = destination_pose->getFilter();
  
 //  return 0;
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
  // cv::Point3f robot_pose;
  // cv::Mat state = kalman_filter->state();
    
  // if (referenceLastUpdated >= NUM_UPDATE_REF_FRAME)
  // {
  //   robot_pose = robot_poses->getFilter();
  //   cv::Mat oldState = kalman_filter->state();
    
  //   float oldVelOther_x = oldState.at<float>(7, 0);
  //   float oldVelOther_y = oldState.at<float>(8, 0);

  //   cv::Mat rnew_T_r0 = xytheta2TransformationMatrix(robot_pose).inv();

  //   cv::Mat r0_v(2, 1, CV_32F);
  //   r0_v.at<float>(0, 0) = oldVelOther_x;
  //   r0_v.at<float>(1, 0) = oldVelOther_y;
  //   cv::Mat rnew_v = rnew_T_r0.rowRange(0, 2).colRange(0, 2) * r0_v;

  //   cv::Mat r0_previous_human_position = cv::Mat::zeros(3, 1, CV_32F);
  //   r0_previous_human_position.at<float>(0, 0) = oldState.at<float>(9, 0);
  //   r0_previous_human_position.at<float>(1, 0) = oldState.at<float>(10, 0);
  //   cv::Mat rnew_previous_human_position = rnew_T_r0 * r0_previous_human_position;


  //   robot_poses->addPoint(cv::Point3f(0, 0, 0));
  //   robot_prev_pose = cv::Point3f(0, 0, 0);
  //   referenceLastUpdated = 0;
  //   robot_pose = robot_prev_pose;


  //   cv::Mat newState = cv::Mat::zeros(11, 1, CV_32F);
  //   newState.at<float>(3, 0) = oldState.at<float>(3, 0);
  //   newState.at<float>(4, 0) = oldState.at<float>(4, 0);
  //   newState.at<float>(5, 0) = human_relative_pose.x;
  //   newState.at<float>(6, 0) = human_relative_pose.y;
  //   newState.at<float>(7, 0) = rnew_v.at<float>(0, 0);
  //   newState.at<float>(8, 0) = rnew_v.at<float>(1, 0);
  //   newState.at<float>(9, 0) = rnew_previous_human_position.at<float>(0, 0);
  //   newState.at<float>(10, 0) = rnew_previous_human_position.at<float>(1, 0);
  
  //   kalman_filter->init(0, newState);

  //   state = kalman_filter->state();

  //   // read the robot global position here as that is going to be our origin
  //   try
  //   {
  //     tf_listener.lookupTransform("robot_0/odom", "robot_0/base_footprint",  
  //                                 ros::Time(0), local_transform);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s", ex.what());
  //     exit(0);
  //     // ros::Duration(1.0).sleep();
  //   }
  // }
  // else
  // {
  //   double v_odom = msg.twist.twist.linear.x;  
  //   double omega_odom = msg.twist.twist.angular.z ;  

  //   double omega_cmd_vel = robot_prev_cmd_vel.z;
  //   double v_cmd_vel = robot_prev_cmd_vel.x;

  //   // measurement
  //   cv::Mat y = cv::Mat(4, 1, CV_32F);
  //   y.at<float>(0, 0) = v_odom;
  //   y.at<float>(1, 0) = omega_odom;
  //   y.at<float>(2, 0) = human_relative_pose.x;
  //   y.at<float>(3, 0) = human_relative_pose.y;

  //   // control input
  //   cv::Mat u = cv::Mat(2, 1, CV_32F);
  //   u.at<float>(0, 0) = v_cmd_vel;
  //   u.at<float>(1, 0) = omega_cmd_vel;

  //   if (!kalman_filter->isInitialized())
  //   {
  //     cv::Mat initState = cv::Mat::zeros(NUM_STATES, 1, CV_32F);
  //     initState.at<float>(5, 0) = human_poses->getFilter().x;
  //     initState.at<float>(6, 0) = human_poses->getFilter().y;
  //     initState.at<float>(9, 0) = human_poses->getFilter().x;
  //     initState.at<float>(10, 0) = human_poses->getFilter().y;

  //     kalman_filter->init(0, initState);
  //   }
    
  //   kalman_filter->update(y, 0.1, u);

  //   robot_pose.x = state.at<float>(0, 0);
  //   robot_pose.y = state.at<float>(1, 0);
  //   robot_pose.z = state.at<float>(2, 0);
  //   robot_pose.z = atan2(sin(robot_pose.z), cos(robot_pose.z));
  //   robot_poses->addPoint(robot_pose);
    
  // }
  // // TODO: MAKE THE TFs ROS PARAM!!!
  // // broadcast the local frame with respect to the global frame
  // local_transform.child_frame_id_ = "local_frame"; // source
  // local_transform.frame_id_ = "robot_0/odom"; // target
  // local_transform.stamp_ = ros::Time::now();
  // tf_broadcaster.sendTransform(local_transform);

  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(robot_pose.x, robot_pose.y, 0.0) );
  // tf::Quaternion q;
  // q.setRPY(0, 0, robot_pose.z);
  // transform.setRotation(q);
  // tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "robot"));

  // // ROS_INFO("Robot Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.z);
  // cv::Point3f avg_dest = destination_pose->getFilter();

  // transform.setOrigin( tf::Vector3(avg_dest.x, avg_dest.y, 0.0) );
  // q.setRPY(0, 0, avg_dest.z);
  // transform.setRotation(q);
  // tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "destination"));

  // transform.setOrigin( tf::Vector3(state.at<float>(5, 0), state.at<float>(6, 0), 0.0) );
  // q.setRPY(0, 0, 0);
  // transform.setRotation(q);
  // tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "person"));

  // cv::Point3f avgDestinations;
  // if (calculateDistination(avgDestinations))
  // {
  //   ROS_ERROR("calculation error");
  //   return;
  // }

  // publishCmdVel(avgDestinations);

  
  // referenceLastUpdated++;
}

