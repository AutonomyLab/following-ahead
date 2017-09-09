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
#include <fstream>

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
  VisualizeTrajectory(ros::NodeHandle nh) 
    : nh_(nh),
      trajectory_fs_("trajecty_fs.txt", std::ofstream::out)
  {
    nh_.param("base_frame", base_frame_, std::string("base_link"));
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("map_frame", map_frame_, std::string("map"));
    nh_.param("person_frame", person_frame_, std::string("person_raw"));

    blob_sub_.subscribe(nh_, "/person_follower/groundtruth_pose", 10);
    blob_tf_filter_ = new tf::MessageFilter<geometry_msgs::TransformStamped>(blob_sub_, tf_listener_, base_frame_, 10);
    blob_tf_filter_->registerCallback( boost::bind(&VisualizeTrajectory::blobCallback, this, _1) );

    odom_topic_subscriber_ = nh_.subscribe("/odom", 1, &VisualizeTrajectory::odometryCallback, this);
  }

  ~VisualizeTrajectory()
  {
    trajectory_fs_.close();
  }

  void blobCallback(const boost::shared_ptr<const geometry_msgs::TransformStamped>& msg)
  {
    // ROS_INFO("blobCallback");
  }

  void odometryCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg)
  {
    tf::StampedTransform map_T_base_frame;
    tf::StampedTransform map_T_person_frame;
    try
    {
      
      tf_listener_.lookupTransform(base_frame_, map_frame_,
                                ros::Time(0), // msg->header.stamp, 
                                map_T_base_frame);
      
      tf_listener_.lookupTransform(person_frame_, map_frame_,
                                  ros::Time(0), // msg->header.stamp, 
                                  map_T_person_frame);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("TF error in odom update %s", ex.what());
      return;
      // ros::Duration(1.0).sleep();
    }

    trajectory_fs_  << map_T_base_frame.inverse().getOrigin().getX() << " "
                    << map_T_base_frame.inverse().getOrigin().getY() << " "
                    << map_T_person_frame.inverse().getOrigin().getX() << " "
                    << map_T_person_frame.inverse().getOrigin().getY() << " "
                    << std::endl;
  }

private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> blob_sub_;
  tf::TransformListener tf_listener_; 
  tf::MessageFilter<geometry_msgs::TransformStamped> * blob_tf_filter_;
  ros::Subscriber odom_topic_subscriber_;

  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string person_frame_;

  std::ofstream trajectory_fs_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize_trajectory"); 
  ros::NodeHandle n("~");

  VisualizeTrajectory visualize_trajectory(n);
  ros::spin();

  return 0;
}