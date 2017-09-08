#include "ros/ros.h"
// #include "yolo2/ImageDetections.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include "robot.hpp"
#include "utils.hpp"

#include <sstream>

#include <limits> 
#include <vector> 
#include <iostream> 
#include <cassert>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class VisualizeTrajectory
{
public:
  VisualizeTrajectory(ros::NodeHandle nh) : nh_(nh)
  {
    nh_.param("base_frame", base_frame_, "base_frame");
    nh_.param("odom_frame", odom_frame_, "odom_frame");
    nh_.param("map_frame", map_frame_, "map_frame");
    nh_.param("person_frame", person_frame_, "person_frame");

    blob_sub_.subscribe(n, "/person_follower/groundtruth_pose", 10);
    blob_tf_filter_ = new tf::MessageFilter<geometry_msgs::TransformStamped>(groundtruth_sub_, tf_listener_, base_frame_, 10);
    blob_tf_filter_->registerCallback( boost::bind(&VisualizeTrajectory::blobCallback, this, _1) );

    odom_topic_subscriber_ = n.subscribe("/husky/odom", 1, &VisualizeTrajectory::odometryCallback, this);
  }

  void blobCallback(const boost::shared_ptr<const geometry_msgs::TransformStamped>& msg)
  {
    ROS_INFO("blobCallback");
  }

  void odometryCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg)
  {
    ROS_INFO("odometryCallback");
  }

private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> blob_sub_;
  tf::TransformListener tf_listener_; 
  tf::MessageFilter<geometry_msgs::TransformStamped> * blob_tf_filter_;

  ros::Subscriber odom_topic_subscriber_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize_trajectory"); 
  ros::NodeHandle n("~");

  VisualizeTrajectory visualize_trajectory(n);
  ros::spin();

  return 0;
}