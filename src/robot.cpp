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

Robot::Robot( ros::NodeHandle n,
              std::string base_frame, std::string odom_frame, 
              std::string map_frame, std::string person_frame, bool use_deadman  )
  : base_frame_(base_frame), odom_frame_(odom_frame), 
    map_frame_(map_frame), person_frame_(person_frame), use_deadman_(use_deadman)
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
  pub_particles_ = n.advertise<sensor_msgs::PointCloud>("person_particle", 1);

  // create actionlib client and tell it to spin a thread for that  
  move_base_client_ptr_ = new MoveBaseClient("move_base", true);
  while(!move_base_client_ptr_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  odom_sub_.subscribe(n, "/husky/odom", 1);
  tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_listener_, map_frame_, 1);
  tf_filter_->registerCallback( boost::bind(&Robot::odometryCallback, this, _1) );  

  if (use_deadman_)
    isDeadManActive = false;
  else
    isDeadManActive = true;

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

Robot::~Robot()
{
  delete move_base_client_ptr_;
}

void Robot::joyCallback(const sensor_msgs::Joy& msg)
{
  if (msg.buttons[0]==1 || !use_deadman_) { // && msg.buttons[6]==1 && msg.buttons[7]==1 ) {
    isDeadManActive = true;
  }
  else 
    isDeadManActive = false;
  // std::cout << "isDeadManActive: " << isDeadManActive << std::endl; 
}

void Robot::myBlobUpdate(const boost::shared_ptr<const geometry_msgs::TransformStamped>& msg)
{
  current_relative_pose_ = *msg;

  tf::StampedTransform r0_T_map; 
  try
  {
    
    tf_listener_.lookupTransform(base_frame_, map_frame_,
                                ros::Time(0), r0_T_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error %s", ex.what());
    return;
    // ros::Duration(1.0).sleep();
  }

  tf::Transform transform_r0_r1;
  transform_r0_r1.setOrigin( tf::Vector3(current_relative_pose_.transform.translation.x, current_relative_pose_.transform.translation.y, 0.0) );
  transform_r0_r1.setRotation(r0_T_map.getRotation());
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform_r0_r1, ros::Time::now(), base_frame_, person_frame_));

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



void Robot::odometryCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg)
{
  current_odometry_ = *msg;

  if  ( 
        fabs(
          current_odometry_.header.stamp.toSec() - current_relative_pose_.header.stamp.toSec()
        ) > 0.15
      )
  {
    // blob not updated, so return
    return;
  }

  human_relative_pose = cv::Point3f(current_relative_pose_.transform.translation.x, current_relative_pose_.transform.translation.y, 0);

  tf::StampedTransform r0_T_map; 
  try
  {
    
    tf_listener_.lookupTransform(base_frame_, map_frame_,
                                ros::Time(0), r0_T_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error %s", ex.what());
    return;
    // ros::Duration(1.0).sleep();
  }
  
  if (!particle_filter_.isInitialized())
  {
    particle_filter_.init(
      NUM_PARTICLES, BLOB_NOISE_STDDEV_X, BLOB_NOISE_STDDEV_Y, PARTICLE_STOCHASTIC_VELOCITY_STDDEV,
      human_relative_pose, r0_T_map,
      base_frame_, map_frame_ 
    );
  }
  else
  {
    // update the damn particles here based on measurement
    particle_filter_.update(human_relative_pose, r0_T_map);
  }




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
  
  // THIS IS NOT RIGHT, THE MEASUREMENTS ARE ABSOLUTE POSE, NOT RELATIVE
  // the z in the relative pose is actually the depth (range). Lol!!!
  // float human_distance = human_relative_pose.z;
  // float human_distance_2 = human_distance * human_distance;
  // // the quaternion x is the bearing angle
  float bearing_angle = current_relative_pose_.transform.rotation.x;
  float bearing_range = current_relative_pose_.transform.translation.z;

  tf::Transform map_T_r0 = r0_T_map.inverse();
  float robot_orientation = tf::getYaw(map_T_r0.getRotation());
  float robot_x = map_T_r0.getOrigin().getX();
  float robot_y = map_T_r0.getOrigin().getY();

  float cos_bearing = cos(bearing_angle);
  float sin_bearing = sin(bearing_angle);
  float cos_robot_orientation = cos(robot_orientation);
  float sin_robot_orientation = sin(robot_orientation);

  // variances
  float robot_orientation_variance = fabs(ROBOT_ORIENTATION_VARIANCE_SCALING * current_odometry_.twist.twist.angular.z);
  float robot_x_variance = fabs(ROBOT_VELOCITY_VARIANCE_SCALING * cos_robot_orientation * current_odometry_.twist.twist.linear.x);
  float robot_y_variance = fabs(ROBOT_VELOCITY_VARIANCE_SCALING * sin_robot_orientation * current_odometry_.twist.twist.linear.x);


  cv::Mat J_range = cv::Mat::zeros(2, 1, CV_32F);
  cv::Mat J_bearing = cv::Mat::zeros(2, 1, CV_32F);
  cv::Mat J_robot_orientation = cv::Mat::zeros(2, 1, CV_32F);
  cv::Mat J_robot_x = cv::Mat::zeros(2, 1, CV_32F);
  cv::Mat J_robot_y = cv::Mat::zeros(2, 1, CV_32F);
  
  J_range.at<float>(0, 0) = -cos_bearing * cos_robot_orientation - sin_bearing * sin_robot_orientation;
  J_range.at<float>(1, 0) = -cos_bearing * sin_robot_orientation + sin_bearing * cos_robot_orientation;

  J_bearing.at<float>(0, 0) = bearing_range * sin_bearing * cos_robot_orientation - bearing_range * cos_bearing * sin_robot_orientation;
  J_bearing.at<float>(1, 0) = bearing_range * sin_bearing * sin_robot_orientation + bearing_range * cos_bearing * cos_robot_orientation;

  J_robot_orientation.at<float>(0, 0) = bearing_range * cos_bearing * sin_robot_orientation - bearing_range * sin_bearing * cos_robot_orientation;
  J_robot_orientation.at<float>(1, 0) = -bearing_range * cos_bearing * cos_robot_orientation - bearing_range * sin_bearing * sin_robot_orientation;

  J_robot_x.at<float>(0, 0) = 1;
  J_robot_x.at<float>(1, 0) = 0;

  J_robot_y.at<float>(0, 0) = 0;
  J_robot_y.at<float>(1, 0) = 1;

  cv::Mat R = cv::Mat::zeros(2, 2, CV_32F);
  R = J_range * J_range.t() * BEARING_RANGE_ERROR_VAR +
      J_bearing * J_bearing.t() * BEARING_ANGLE_ERROR_VAR +
      J_robot_orientation * J_robot_orientation.t() * robot_orientation_variance +
      J_robot_x * J_robot_x.t() * robot_x_variance +
      J_robot_y * J_robot_y.t() * robot_y_variance;

  R.at<float>(0, 0) += 0.01;
  R.at<float>(1, 1) += 0.01;

  
  person_kalman_->update(y, LOOP_TIME, R);
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

  tf_broadcaster_.sendTransform(person_kalman_transform);

  // ------------------------ Predict position ------------------------ //
  tf::StampedTransform prediction_kalman_transform;
  prediction_kalman_transform.child_frame_id_ = "prediction_kalman"; // source
  prediction_kalman_transform.frame_id_ = map_frame_; // target
  prediction_kalman_transform.stamp_ = ros::Time::now();

  float prediction_x = est_x + cos(est_theta) * PREDICTION_LOOKAHEAD_DISTANCE;
  float prediction_y = est_y + sin(est_theta) * PREDICTION_LOOKAHEAD_DISTANCE;

  prediction_kalman_transform.setOrigin(tf::Vector3(prediction_x, prediction_y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, est_theta);
  prediction_kalman_transform.setRotation(q);

  tf_broadcaster_.sendTransform(prediction_kalman_transform);
  cv::Mat person_error_covariance = person_kalman_->getStateErrorCovariance();
  float velocity_stddev = sqrt(person_error_covariance.at<float>(VEL_IDX, VEL_IDX));
  float theta_stddev = sqrt(person_error_covariance.at<float>(THETA_IDX, THETA_IDX));
  float target_x_stddev = sqrt(person_error_covariance.at<float>(X_T_IDX, X_T_IDX));
  float target_y_stddev = sqrt(person_error_covariance.at<float>(Y_T_IDX, Y_T_IDX));
  // tell the robot to go to the predicted position
  if (
    true // est_v > DISTANCE_EPSILON/10
    // theta_stddev < ORIENTATION_ERROR_EPSILON
    // velocity_stddev < VELOCITY_ERROR_EPSILON &&
    // target_x_stddev < POSITION_ERROR_EPSILON &&
    // target_y_stddev < POSITION_ERROR_EPSILON
  )
  {
    cv::Point3f target_position(est_x, est_y, 0);
    cv::Point3f target_position_stddev(
      target_x_stddev, target_y_stddev, 0
    );

    if (!prediction_particle_filter_.isInitialized())
    {
      prediction_particle_filter_. init(
        NUM_PARTICLES, 
        target_position,
        est_v, velocity_stddev,
        est_theta, theta_stddev
      );
    }
    else
    {
      prediction_particle_filter_.update(
        target_position, target_position_stddev, 
        est_v, velocity_stddev,
        est_theta, theta_stddev,
        LOOP_TIME
      );
    }

    // lets publish particles
    sensor_msgs::PointCloud particles;
    particles.header.stamp = ros::Time::now();
    particles.header.frame_id = map_frame_;
    
    for (size_t i = 0, len = prediction_particle_filter_.getNumPredictionParticles(); i < len; i++)
    {
      PredictionParticleFilter::PredictionParticle particle = prediction_particle_filter_.getPredictionParticleAt(i);
      cv::Point3f particle_state = particle.getState();

      geometry_msgs::Point32 point;
      point.x = particle_state.x;
      point.y = particle_state.y;
      point.z = particle.getWeight();
      particles.points.push_back(point);

      sensor_msgs::ChannelFloat32 color;
      color.name = "rgb";
      color.values.push_back((float)rand()/RAND_MAX*255);
      color.values.push_back(0.0);
      color.values.push_back(0.0);

      if (fabs(point.x) < 0.001 && fabs(point.y) < 0.001)
      {
         ROS_INFO("Particle %d at origin weight:%f ", i, point.z);
      }
    }
    
    pub_particles_.publish(particles);


    cv::Point3f prediction_global(prediction_x, prediction_y, 0);
    cv::Point3f prediction_local = transformPoint(r0_T_map, prediction_global);

    if (isDeadManActive && 
        prediction_local.x > 0 // don't follow if robot is behind
    )
    {
      move_base_msgs::MoveBaseGoal nav_goal_msg;

      nav_goal_msg.target_pose.header.stamp = ros::Time::now();
      nav_goal_msg.target_pose.header.frame_id = map_frame_;
      nav_goal_msg.target_pose.pose.position.x = prediction_x;
      nav_goal_msg.target_pose.pose.position.y = prediction_y;
      nav_goal_msg.target_pose.pose.position.z = 0;
      nav_goal_msg.target_pose.pose.orientation.x = q.x();
      nav_goal_msg.target_pose.pose.orientation.y = q.y();
      nav_goal_msg.target_pose.pose.orientation.z = q.z();
      nav_goal_msg.target_pose.pose.orientation.w = q.w();
      move_base_client_ptr_->sendGoal(nav_goal_msg);
    }
    else // if (!isDeadManActive)
    {
      // cancel anything that going on
      actionlib::SimpleClientGoalState move_base_state = move_base_client_ptr_->getState();
      if (
          move_base_state == actionlib::SimpleClientGoalState::ACTIVE ||
          move_base_state == actionlib::SimpleClientGoalState::PENDING
      )
      {
        ROS_INFO("Cancelling move_base goals");
        move_base_client_ptr_->cancelGoal();
      }
    }
  }
  else
  {
    ROS_WARN(
      "High error: %f, %f, %f, %f", 
      velocity_stddev, theta_stddev, target_x_stddev, target_y_stddev
    );
  }

  // // ROS_INFO("Robot Pose: %f, %f, %f", robot_pose.x, robot_pose.y, robot_pose.z);
  // cv::Point3f avg_dest = destination_pose->getFilter();

  // transform.setOrigin( tf::Vector3(avg_dest.x, avg_dest.y, 0.0) );
  // q.setRPY(0, 0, avg_dest.z);
  // transform.setRotation(q);
  // tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "destination"));

  // transform.setOrigin( tf::Vector3(state.at<float>(5, 0), state.at<float>(6, 0), 0.0) );
  // q.setRPY(0, 0, 0);
  // transform.setRotation(q);
  // tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_frame", "person"));

  // cv::Point3f avgDestinations;
  // if (calculateDistination(avgDestinations))
  // {
  //   ROS_ERROR("calculation error");
  //   return;
  // }

  // publishCmdVel(avgDestinations);
  
}

