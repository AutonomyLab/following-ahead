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
#include <costmap_2d/costmap_2d_ros.h>
#include "utils.hpp"

#include <set>
#include <exception>


Robot::Robot( ros::NodeHandle n,
              std::string base_frame, std::string odom_frame, 
              std::string map_frame, std::string person_frame, bool use_deadman  )
  : base_frame_(base_frame), odom_frame_(odom_frame), 
    map_frame_(map_frame), person_frame_(person_frame), use_deadman_(use_deadman),
    image_transport_(n),last_human_pose_update(ros::Time::now().toSec()),
    person_orientation_filter(PERSON_ORIENTATION_FILTER_SIZE)
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

  n.param("blob_measurement_timeout", blob_measurements_timeout_, 10.0);

  cmd_vel_publisher =  n.advertise<geometry_msgs::Twist>("/person_follower/zero_cmd_vel", 1);
  pub_particles_ = n.advertise<sensor_msgs::PointCloud>("person_particle", 1);
  
  // create actionlib client and tell it to spin a thread for that  
  move_base_client_ptr_ = new MoveBaseClient("move_base", true);
  while(!move_base_client_ptr_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // tf_odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_filtered_sub_, tf_listener_, base_frame_, 10);
  // tf_odom_filter_->registerCallback( boost::bind(&Robot::odometryCallback, this, _1) );
  
  odom_topic_subscriber_ = n.subscribe("/husky/odom", 1, &Robot::odometryCallback, this);
  map_image_pub_ = image_transport_.advertise("map_image", 1);
  map_image_pub_temp_ = image_transport_.advertise("map_image_temp", 1);

  map_service_client_ = n.serviceClient<nav_msgs::GetMap>("/dynamic_map", true);

  if (use_deadman_)
    isDeadManActive = false;
  else
    isDeadManActive = true;

	human_prev_pose = cv::Point3f(0,0,0);
  human_relative_pose = cv::Point3f(0,0,0);
  human_prev_vel = cv::Point3f(0,0,0);
  robot_prev_cmd_vel = cv::Point3f(0,0,0);

  robot_prev_pose = cv::Point3f(0,0,0);
  robot_prev_degree_dest = 0.0;
  referenceLastUpdated = 0;
	pid_turn.set(AVOID_TURN, -AVOID_TURN, AVOID_TURN, 0.0100, 0.01);
  pid_cruse.set(CRUISE_SPEED, -CRUISE_SPEED, -CRUISE_SPEED  , .0100, 0.001);

  // zero time indicates invalid pose
  current_relative_pose_.header.stamp = ros::Time(0);
  previous_relative_pose_.header.stamp = ros::Time(0);
}

Robot::~Robot()
{
  delete move_base_client_ptr_;
}

void Robot::publishZeroCmdVel()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.z = 1; // make it fly :)
  cmd_vel.angular.z = 0;
  cmd_vel_publisher.publish(cmd_vel);
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

  // r0 is the camera frame for real robot and base_link for simulation
  tf::StampedTransform r0_T_map; 
  tf::StampedTransform base_frame_T_map;
  try
  {
    
    tf_listener_.lookupTransform(current_relative_pose_.header.frame_id, map_frame_,
                                msg->header.stamp,// ros::Time(0), 
                                r0_T_map);
    tf_listener_.lookupTransform(base_frame_, map_frame_,
                                msg->header.stamp,//ros::Time(0), 
                                base_frame_T_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error in blob update %s", ex.what());
    return;
    // ros::Duration(1.0).sleep();
  }

  tf::Transform transform_r0_human;
  transform_r0_human.setOrigin( tf::Vector3(
    current_relative_pose_.transform.translation.x, 
    current_relative_pose_.transform.translation.y, 
    current_relative_pose_.transform.translation.z
  ));
  transform_r0_human.setRotation(r0_T_map.getRotation());
  
  absolute_tf_pose_human_ = r0_T_map.inverse() * transform_r0_human;
  // project to ground plane
  absolute_tf_pose_human_.setOrigin(tf::Vector3(
    absolute_tf_pose_human_.getOrigin().getX(),
    absolute_tf_pose_human_.getOrigin().getY(),
    0
  ));
  absolute_tf_pose_human_.setRotation(tf::Quaternion(0, 0, 0, 1));
  
  // tf::Transform transform_base_frame_human = base_frame_T_map * absolute_tf_pose_human_;
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(
      absolute_tf_pose_human_, // transform_base_frame_human, 
      ros::Time::now(), 
      map_frame_, // base_frame_, 
      person_frame_
    )
  );

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


void Robot::mapCallback(nav_msgs::OccupancyGrid &map_msg)
{
  map_image_ = cv::Mat(map_msg.info.width, map_msg.info.height, CV_8UC1, (void*)map_msg.data.data());
  map_occupancy_grid_ = map_msg;

  // all the unknown spaces are free
  uint8_t *data = map_image_.data;
  for (size_t i = 0; i < map_image_.total(); i++, data++)
  {
    if (*data == 255)
    {
      *data = 0;
    }
  }
  cv::Mat map_image_temp;
  
  // ------------------ Remove these if thresholding is not needed ------------------ //
  cv::threshold(map_image_, map_image_temp, OCCUPANCY_THRESHOLD, 255, CV_THRESH_BINARY);
  map_image_ = map_image_temp;
  // ------------------ Remove these if thresholding is not needed ------------------ //

  // dilation to inflate the obstacles (and fill holes)
  // publish the undilated image (for debugging)
  cv::Mat debug_map_flipped;
  cv::flip(map_image_, debug_map_flipped, 0);

  cv_bridge::CvImage cv_ptr;
  cv_ptr.image = debug_map_flipped;
  cv_ptr.encoding = "mono8";
  cv_ptr.header = map_occupancy_grid_.header;
  map_image_pub_temp_.publish(cv_ptr.toImageMsg());

  int dilation_size = round((double)OBSTACLE_INFLATION  / map_msg.info.resolution);
  cv::Mat dilation_element = cv::getStructuringElement( 
    cv::MORPH_RECT,
    cv::Size(dilation_size, dilation_size)
  );
  dilate(map_image_, map_image_temp, dilation_element);

  cv::Mat erosion_element = cv::getStructuringElement( 
    cv::MORPH_RECT,
    cv::Size(dilation_size * 0.5, dilation_size * 0.5)
  );
  erode(map_image_temp, map_image_, dilation_element);
  

}

cv::Point3f Robot::updatePrediction()
{
  // TODO: fill holes
  // TODO: account for map orientation

  cv::Point person_image_coordinates(
    (absolute_tf_pose_human_.getOrigin().x() - (int)map_occupancy_grid_.info.origin.position.x) / map_occupancy_grid_.info.resolution,
    (absolute_tf_pose_human_.getOrigin().y() - (int)map_occupancy_grid_.info.origin.position.y) / map_occupancy_grid_.info.resolution
  );

  cv::Point robot_image_coordinates(
    (absolute_tf_pose_robot_.getOrigin().x() - (int)map_occupancy_grid_.info.origin.position.x) / map_occupancy_grid_.info.resolution,
    (absolute_tf_pose_robot_.getOrigin().y() - (int)map_occupancy_grid_.info.origin.position.y) / map_occupancy_grid_.info.resolution
  );  
  
  cv::Point prediction_image_coordinates(
    (prediction_global_.x - (int)map_occupancy_grid_.info.origin.position.x) / map_occupancy_grid_.info.resolution,
    (prediction_global_.y - (int)map_occupancy_grid_.info.origin.position.y) / map_occupancy_grid_.info.resolution
  );

  // check if the robot can go to the prediction 
  cv::LineIterator robot_prediction_line_iterator(
    map_image_, 
    cv::Point(round(robot_image_coordinates.x), round(robot_image_coordinates.y)),
    cv::Point(round(prediction_image_coordinates.x), round(prediction_image_coordinates.y))
  );

  cv::Mat debug_map = map_image_.clone();
  
  // bool is_robot_to_prediction_feasible = true;
  // cv::LineIterator it = robot_prediction_line_iterator;
  // for (size_t i = 0; i < robot_prediction_line_iterator.count; i++, it++)
  // {
  //   if (map_image_.at<uint8_t>(it.pos()) == 255)
  //   {
  //     is_robot_to_prediction_feasible = false;
  //     break;
  //   }
  // }

  // if (is_robot_to_prediction_feasible)
  // {
  //   // also check if it is very close to obstacles
  //   cv::Point new_prediction_image_coordinates;
  //   is_robot_to_prediction_feasible = LinearMotionModel::checkObjectDestinationFeasibility(
  //     person_image_coordinates, 
  //     prediction_image_coordinates, 
  //     map_image_, map_occupancy_grid_.info.resolution,
  //     new_prediction_image_coordinates,
  //     debug_map
  //   );

  //   if (is_robot_to_prediction_feasible)
  //   {
  //     // don't update the prediction
  //     prediction_global_prev_ = prediction_global_;
  //     cv::circle(debug_map, person_image_coordinates, 8, 255);
  //     cv::circle(debug_map, prediction_image_coordinates, 5, 255);
  //   }
  //   else
  //   {
  //     ROS_WARN("Destination close to obstacle, running the linear motion model on it");
  //     std::cout << "prediction_image_coordinates: " << prediction_image_coordinates << std::endl;
  //     // increasing the length
  //     prediction_image_coordinates = new_prediction_image_coordinates;
  //     std::cout << "new_prediction_image_coordinates: " << prediction_image_coordinates << std::endl;
      
  //     cv::circle(debug_map, new_prediction_image_coordinates, 15, 255);
  //     is_robot_to_prediction_feasible = false;
    
  //   }
  // }
  
  // if (!is_robot_to_prediction_feasible)
  {
    
    cv::Point new_person_image_coordinates, new_prediction_image_coordinates;
    
    if  ( person_motion_model_.updatePrediction(   
            map_image_, map_occupancy_grid_.info.resolution,
            person_image_coordinates, prediction_image_coordinates, PREDICTION_LOOKAHEAD_DISTANCE, 
            new_person_image_coordinates, new_prediction_image_coordinates,
            debug_map
          ) == 1
        )
    {
      ROS_ERROR("updatePrediction failed");
      return prediction_global_prev_;
    }
    // account for map orientation
    float orientation = atan2(
      new_prediction_image_coordinates.y - new_person_image_coordinates.y,
      new_prediction_image_coordinates.x - new_person_image_coordinates.x
    );
    prediction_global_prev_ = cv::Point3f(
                                new_prediction_image_coordinates.x * map_occupancy_grid_.info.resolution + (int)map_occupancy_grid_.info.origin.position.x,
                                new_prediction_image_coordinates.y * map_occupancy_grid_.info.resolution + (int)map_occupancy_grid_.info.origin.position.y,
                                orientation
                              );
  }
  
  
  

  cv::Mat debug_map_flipped;
  cv::flip(debug_map, debug_map_flipped, 0);

  cv_bridge::CvImage cv_ptr;
  cv_ptr.image = debug_map_flipped;
  cv_ptr.encoding = "mono8";
  cv_ptr.header = map_occupancy_grid_.header;

  map_image_pub_.publish(cv_ptr.toImageMsg()); 

    
    return prediction_global_prev_;
}

int Robot::updateHumanPrediction(float dt)
{ 
  // just use the last known human position
  last_human_pose_update = ros::Time::now().toSec();
  return 0;

  cv::Mat state = person_kalman_->state();
  float person_vel = state.at<float>(VEL_IDX, 0);
  float person_theta = state.at<float>(THETA_IDX, 0);

  if (!std::isfinite(person_vel) || !std::isfinite(person_theta))
  {
    return 1;
  }

  float person_x = absolute_tf_pose_human_.getOrigin().getX();
  float person_y = absolute_tf_pose_human_.getOrigin().getY();

  float prediction_person_x = person_x + cos(person_theta) * dt * person_vel;
  float prediction_person_y = person_y + sin(person_theta) * dt * person_vel;

  absolute_tf_pose_human_previous_ = absolute_tf_pose_human_;
  absolute_tf_pose_human_.setOrigin(
    tf::Vector3(
      prediction_person_x,
      prediction_person_y,
      0
    )
  );
  
  // publish tf of the predicted human pose for visualization   
  tf::StampedTransform base_frame_T_map;
  try
  {
    
    tf_listener_.lookupTransform(base_frame_, map_frame_,
                                ros::Time(0), base_frame_T_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error %s", ex.what());
    return 1;
    // ros::Duration(1.0).sleep();
  }

  // tf::Transform transform_base_frame_human = base_frame_T_map * absolute_tf_pose_human_;
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(
      absolute_tf_pose_human_, // transform_base_frame_human, 
      ros::Time::now(), 
      map_frame_, // base_frame_, 
      person_frame_
    )
  );

  return 0;
}

void Robot::odometryCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg) 
{
  current_odometry_ = *msg;
  spinOnce();
}

void Robot::spinOnce() try
{
  bool is_using_predicted_human = false;
  
  // check if the robot is rotating
  if (current_odometry_.twist.twist.angular.z > 0.15)
  {
    ROS_WARN("Pure rotation detection, not doing updates!!");
    is_using_predicted_human = true;
  }

  // check if the person is opposite of estimated direction
  {
    if (person_kalman_->isInitialized())
    {
      cv::Mat state = person_kalman_->state();
      float est_theta = state.at<float>(THETA_IDX, 0);

      float theta = atan2(
        absolute_tf_pose_human_.getOrigin().getY() - absolute_tf_pose_human_previous_.getOrigin().getY(),
        absolute_tf_pose_human_.getOrigin().getX() - absolute_tf_pose_human_previous_.getOrigin().getX()
      );

      float angle = vectorAngle(
        cv::Point2f(cos(est_theta), sin(est_theta)),
        cv::Point2f(cos(theta), sin(theta))
      );

      if (angle > 120*M_PI/180)
      {
        ROS_ERROR("new point is behind angle: %f",angle*180/M_PI);
        is_using_predicted_human = true;
      }
    }
  }
  
  float dt = current_relative_pose_.header.stamp.toSec() - previous_relative_pose_.header.stamp.toSec();
  // check if we are still getting blob readings
  if (
        current_relative_pose_.header.stamp.toSec() &&  // is valid
        previous_relative_pose_.header.stamp.toSec() && // is valid
        dt <= 0
      )
  {

    if (!isDeadManActive)
    {
      ROS_WARN("Deadman inactive and no blob reading");
      throw OdometryException();
    }

    if  (
          ros::Time::now().toSec() - current_relative_pose_.header.stamp.toSec() > blob_measurements_timeout_
        )
    {
        ROS_WARN("blob measurements timed out, stopping...");  
        throw OdometryException();
    }
    else
    {
      // no new blob measurement, just go to the last destination
      if (last_human_pose_update && person_kalman_->isInitialized())
      {
        ROS_WARN("no new blob measurement, using last estimated velocity for update");
        is_using_predicted_human = true;
      }
      else
      {
        ROS_WARN("no new blob measurement and no prediction for person available, returning...");
        return;
      }
      
      // ROS_WARN("no new blob measurement, returning");
      // return;
    }
  }

  if (is_using_predicted_human)
  {
    float dt = ros::Time::now().toSec() - last_human_pose_update;
    if (dt <= 0)
    {
      ROS_ERROR("dt <= 0");
      return;
    }
    if (updateHumanPrediction(dt))
    {
      ROS_WARN("prediction for person failed, returning...");
      return;   
    }
  }

  
  // if the time between current and previous is too large (when we lose the person), reinitialize the filters 
  if (dt > MAX_DEL_TIME)
  {
    ROS_WARN("dt (%f) large for filter: reinitializing", dt);
    person_kalman_->reintialize();
    prediction_particle_filter_.reintialize();
  }

  // if  ( 
  //       fabs(
  //         current_odometry_.header.stamp.toSec() - current_relative_pose_.header.stamp.toSec()
  //       ) > 0.15
  //     )
  // {
  //   // blob not updated, so return
  //   return;
  // }
  nav_msgs::GetMap get_map_srv;
  
  if (map_service_client_.call(get_map_srv))
  {
    nav_msgs::OccupancyGrid &map_msg(get_map_srv.response.map);
    mapCallback(map_msg); 
  }
  else
  {
    ROS_ERROR("get_map service call failed!!!");
  }

  tf::StampedTransform r0_T_map; 
  // // relative pose of human wrt robot (base_link)
  // tf::StampedTransform r0_T_r1;
  try
  {
    
    tf_listener_.lookupTransform(base_frame_, map_frame_,
                                ros::Time(0), r0_T_map);
    // tf_listener_.lookupTransform(base_frame_, person_frame_,
    //                             ros::Time(0), r0_T_r1);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF error %s", ex.what());
    throw OdometryException();
    // ros::Duration(1.0).sleep();
  }

  // human_relative_pose = cv::Point3f(
  //   r0_T_r1.getOrigin().getX(), 
  //   r0_T_r1.getOrigin().getY(), 
  //   r0_T_r1.getOrigin().getZ()
  // );
  // absolute_tf_pose_robot_ = r0_T_map.inverse();

  float est_x;
  float est_y;
  float est_v;
  float est_theta;

  if (!is_using_predicted_human)
  {
    cv::Point3f robot_pose;
    cv::Mat state = person_kalman_->state();
    
    // global pose
    // cv::Point3f human_global_pose = transformPoint(r0_T_map.inverse(), human_relative_pose);
    // measurement
    cv::Mat y = cv::Mat(2, 1, CV_32F);
    y.at<float>(0, 0) = absolute_tf_pose_human_.getOrigin().getX(); //human_global_pose.x;
    y.at<float>(1, 0) = absolute_tf_pose_human_.getOrigin().getY(); //human_global_pose.y;
  
    bool isKalmanValid = true;
    if (!person_kalman_->isInitialized())
    {
      // check if both the poses are valid
      if (
            current_relative_pose_.header.stamp.toSec() &&
            previous_relative_pose_.header.stamp.toSec()
          )
      {  
        // check if the person has actually moved
        float person_movement = sqrt(
          pow(absolute_tf_pose_human_.getOrigin().getX() - absolute_tf_pose_human_previous_.getOrigin().getX(), 2) +
          pow(absolute_tf_pose_human_.getOrigin().getY() - absolute_tf_pose_human_previous_.getOrigin().getY(), 2)
        );
  
        if (person_movement >= DISTANCE_EPSILON)
        {
          float theta = atan2(
            absolute_tf_pose_human_.getOrigin().getY() - absolute_tf_pose_human_previous_.getOrigin().getY(),
            absolute_tf_pose_human_.getOrigin().getX() - absolute_tf_pose_human_previous_.getOrigin().getX()
          );
          float velocity = fabs(person_movement / dt);

          // check if the person is walking towards the robot
          float robot_theta = tf::getYaw(r0_T_map.inverse().getRotation());

          float robot_human_angle = vectorAngle(
            cv::Point2f(cos(theta), sin(theta)),
            cv::Point2f(cos(robot_theta), sin(robot_theta))
          );

          if (robot_human_angle < 120. * M_PI / 180.)
          {  
            cv::Mat initState = cv::Mat::zeros(NUM_STATES, 1, CV_32F);
            initState.at<float>(X_T_IDX, 0) = absolute_tf_pose_human_.getOrigin().getX();
            initState.at<float>(Y_T_IDX, 0) = absolute_tf_pose_human_.getOrigin().getY();
            initState.at<float>(X_T_1_IDX, 0) = absolute_tf_pose_human_previous_.getOrigin().getX();
            initState.at<float>(Y_T_1_IDX, 0) = absolute_tf_pose_human_previous_.getOrigin().getY();
            initState.at<float>(VEL_IDX, 0) = velocity;
            initState.at<float>(THETA_IDX, 0) = theta;
    
            person_kalman_->init(0, initState);
            person_orientation_filter.initialize(theta);

            isKalmanValid = true;

            ROS_INFO("EKF initialized");
            // ROS_INFO(
            //   "start state: speed- %f, theta- %f", 
            //   initState.at<float>(VEL_IDX, 0),
            //   initState.at<float>(THETA_IDX, 0)
            // );
          }
          else
          {
            ROS_ERROR("Person backing off from robot, not initializing");
            isKalmanValid = false;
          }
        }
        else
        {
          isKalmanValid = false;
        }
      }
      else
      {
        isKalmanValid = false;
      }
    }
    
    previous_relative_pose_ = current_relative_pose_;
    absolute_tf_pose_human_previous_ = absolute_tf_pose_human_;
    last_human_pose_update = ros::Time::now().toSec();  
  
    if (!isKalmanValid)
    {
      person_kalman_->reintialize();
      throw OdometryException();
    }

    float bearing_angle = atan2(human_relative_pose.y, -human_relative_pose.x);
    float bearing_range = sqrt(
      pow(human_relative_pose.x, 2) +
      pow(human_relative_pose.y, 2)
    );
  
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
  
    person_kalman_->update(y, dt, R);
    cv::Mat new_state = person_kalman_->state();
  
    // TODO: MAKE THE TFs ROS PARAM!!!
    // broadcast the estimated person position frame with respect to the global frame
    tf::StampedTransform person_kalman_transform;
    person_kalman_transform.child_frame_id_ = "person_kalman"; // source
    person_kalman_transform.frame_id_ = map_frame_; // target
    person_kalman_transform.stamp_ = ros::Time::now();
    
    est_x = new_state.at<float>(X_T_IDX, 0);
    est_y = new_state.at<float>(Y_T_IDX, 0);
    est_v = new_state.at<float>(VEL_IDX, 0);
    est_theta = new_state.at<float>(THETA_IDX, 0);
  
  
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
  
      // reset the absolute pose based on kalman filter update  
      absolute_tf_pose_human_.setOrigin(
        tf::Vector3(
          est_x,
          est_y,
          0
        )  
      );
      absolute_tf_pose_human_previous_ = absolute_tf_pose_human_;

      person_orientation_filter.addPoint(est_theta);
    }
    else
    {
      ROS_ERROR("Kalman filter returned NaN position");
      throw OdometryException();
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
      throw OdometryException();
    }
  
    tf_broadcaster_.sendTransform(person_kalman_transform);
  }
  else
  {
    previous_relative_pose_ = current_relative_pose_;
    last_human_pose_update = ros::Time::now().toSec();  

    absolute_tf_pose_human_previous_ = absolute_tf_pose_human_;

    // use the last known state
    cv::Mat state = person_kalman_->state();
    est_x = state.at<float>(X_T_IDX, 0);
    est_y = state.at<float>(Y_T_IDX, 0);
    est_v = state.at<float>(VEL_IDX, 0);
    est_theta = state.at<float>(THETA_IDX, 0);
    
  }

  // ------------------------ Predict position ------------------------ //
  tf::StampedTransform prediction_kalman_transform;
  prediction_kalman_transform.child_frame_id_ = "prediction_kalman"; // source
  prediction_kalman_transform.frame_id_ = map_frame_; // target
  prediction_kalman_transform.stamp_ = ros::Time::now();

  // use filtered theta
  // est_theta = person_orientation_filter.getFilter();

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
    // true 
    est_v > DISTANCE_EPSILON*dt
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
        dt
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
    }
    
    pub_particles_.publish(particles);

    prediction_global_.x = prediction_x;
    prediction_global_.y = prediction_y;

    if (map_image_.total())
    {
      prediction_global_ = updatePrediction();

      tf::StampedTransform prediction_updated;
      prediction_updated.child_frame_id_ = "updated_prediction"; // source
      prediction_updated.frame_id_ = map_frame_; // target
      prediction_updated.stamp_ = ros::Time::now();

      prediction_updated.setOrigin(tf::Vector3(prediction_global_.x, prediction_global_.y, 0));
      tf::Quaternion q;
      q.setRPY(0, 0, prediction_global_.z);
      prediction_updated.setRotation(q);

      tf_broadcaster_.sendTransform(prediction_updated);
    }

    prediction_local_ = transformPoint(r0_T_map, prediction_global_);
    // angle of the prediction with respect to the robots x axis
    double prediction_angle = atan2(prediction_local_.y, prediction_local_.x);
    if  (
          // true
          isDeadManActive && 
          fabs(prediction_angle) < 130. * M_PI / 180.  // don't follow if robot is behind
        )
    {
      sendNavGoal();
    }
    else // if (!isDeadManActive)
    {
      // cancel anything that's going on
      ROS_WARN("deadman off or goal behind the robot, stopping");
      // person_kalman_->reintialize();
      // prediction_particle_filter_.reintialize();
      throw OdometryException();
    }
  }
  else
  {
    // sendNavGoal();
    ROS_WARN("Person velocity low, going to last destination");
  }
  
}
catch(OdometryException &e)
{
  if (!isDeadManActive)
    publishZeroCmdVel();
}

void Robot::sendNavGoal()
{
  static double nav_goal_last_sent = 0;
  
  tf::Quaternion q;
  q.setRPY(0, 0, prediction_global_.z);

  if (ros::Time::now().toSec() - nav_goal_last_sent > 1.0)
  {
    move_base_msgs::MoveBaseGoal nav_goal_msg;

    nav_goal_msg.target_pose.header.stamp = ros::Time::now();
    nav_goal_msg.target_pose.header.frame_id = map_frame_;
    nav_goal_msg.target_pose.pose.position.x = prediction_global_.x;
    nav_goal_msg.target_pose.pose.position.y = prediction_global_.y;
    nav_goal_msg.target_pose.pose.position.z = 0;
    nav_goal_msg.target_pose.pose.orientation.x = q.x();
    nav_goal_msg.target_pose.pose.orientation.y = q.y();
    nav_goal_msg.target_pose.pose.orientation.z = q.z();
    nav_goal_msg.target_pose.pose.orientation.w = q.w();
    move_base_client_ptr_->sendGoal(nav_goal_msg);

    nav_goal_last_sent = ros::Time::now().toSec();
  } 
}

