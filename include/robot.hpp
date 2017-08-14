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
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/bind.hpp>
#include "person_kalman.hpp"
#include "ParticleFilter.hpp"
#include "PredictionParticleFilter.hpp"
#include "linear_motion_model.hpp"

class Robot
{
private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
    
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::StampedTransform local_transform_;
    tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    ros::Subscriber odom_topic_subscriber_;
    
    MoveBaseClient *move_base_client_ptr_;

    nav_msgs::Odometry current_odometry_;
    geometry_msgs::TransformStamped current_relative_pose_;
    tf::Transform absolute_tf_pose_human_;

    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string person_frame_;

    ros::ServiceClient map_service_client_;

    // temporary stuffs
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher map_image_pub_;
    image_transport::Publisher map_image_pub_temp_;

    cv::Point3f prediction_local_;
    cv::Point3f prediction_global_;
    LinearMotionModel person_motion_model_;

    cv::Mat map_image_;
    nav_msgs::OccupancyGrid map_occupancy_grid_;
public:
    Robot(  ros::NodeHandle n, 
            std::string base_frame, std::string odom_frame, 
            std::string map_frame, std::string person_frame, bool use_deadman   );

    ~Robot();
    
    void joyCallback(const sensor_msgs::Joy& msg);
    void odometryCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg);
    cv::Point3f getRobotPose();
    cv::Point3f getHumanPose();
    void myBlobUpdate (const boost::shared_ptr<const geometry_msgs::TransformStamped>& msg);
    int relativePoseCallback();
    void mapCallback(nav_msgs::OccupancyGrid &map_msg);
    cv::Point3f updatePrediction();

    int kiniectPoseCallback();
    int calculateDistination(cv::Point3f&);
    int publishCmdVel(cv::Point3f destination);
};


#endif  