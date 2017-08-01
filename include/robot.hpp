#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "filter.hpp"
#include "pid.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include "person_kalman.hpp"
#include "ParticleFilter.hpp"
#include "PredictionParticleFilter.hpp"

class Robot
{
private:
    int time;
    ros::Publisher cmd_vel_publisher;
    tf::TransformListener listener;
    int referenceLastUpdated;
    Filter* robot_poses;
    Filter* human_poses;
    Filter* destination_pose;
    ros::Publisher pub_waypoints_;
    ros::Publisher pub_particles_;
    double robot_prev_degree_dest;
    bool use_deadman_;
    cv::Point3f robot_prev_pose;
    cv::Point3f human_prev_pose;
    cv::Point3f human_relative_pose;
    cv::Point3f robot_prev_cmd_vel;
    cv::Point3f human_prev_vel;
    PID pid_turn;
    PID pid_cruse;
    bool isDeadManActive;

    ParticleFilter particle_filter_;
    PredictionParticleFilter prediction_particle_filter_;
    PersonKalman *person_kalman_;
    
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform local_transform;

    ros::Publisher pub_nav_goal_;

    nav_msgs::Odometry current_odometry_;

    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string person_frame_;
    
public:
    Robot(  ros::NodeHandle n, 
            std::string base_frame, std::string odom_frame, 
            std::string map_frame, std::string person_frame, bool use_deadman   );

    void joyCallback(const sensor_msgs::Joy& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);
    cv::Point3f getRobotPose();
    cv::Point3f getHumanPose();
    void myBlobUpdate (const geometry_msgs::TransformStamped& msg);
    int relativePoseCallback();
    int kiniectPoseCallback();
    int calculateDistination(cv::Point3f&);
    int publishCmdVel(cv::Point3f destination);
};


#endif  