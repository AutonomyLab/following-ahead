#define CAM_WRT_LASER_X -0.075
#define CAM_WRT_LASER_Y -0.01
#define CAM_WRT_LASER_Z 0.174

#define CAM_WRT_LASER_PITCH 16.5

#include "ros/ros.h"
#include "yolo2/ImageDetections.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>

#include "robot.hpp"
#include "utils.hpp"
#include "ocamcalib/ocam_functions.h"

#include <sstream>

#include <limits> 
#include <vector> 
#include <iostream> 
#include <cassert>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class FisheyeBlobPublisher
{
private:
  ros::Publisher pubPointCloud_;
  ros::Publisher pubRelativePose_;
  sensor_msgs::PointCloud::Ptr pointCloudMsg_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  cv::Point3f human_prev_pose_;
  float camera_wrt_laser_x_;
  float camera_wrt_laser_y_;
  float camera_wrt_laser_z_;
  float camera_wrt_laser_pitch_;
  bool track_person_;
  std::string camera_parent_frame_;
  std::string fisheye_config_file_;

  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_undistorted_image_;

  struct ocam_model fisheye_model_;

public:
  FisheyeBlobPublisher(ros::NodeHandle n)
    : nh_(n),
      pointCloudMsg_(new sensor_msgs::PointCloud),
      image_transport_(n)
  {
    track_person_ = false;
    pubPointCloud_ =  nh_.advertise<sensor_msgs::PointCloud>("person_cloud", 1);
    pubRelativePose_ = nh_.advertise<geometry_msgs::TransformStamped>("/person_follower/groundtruth_pose", 1);

    pub_undistorted_image_ = image_transport_.advertise("/camera/fisheye/undistorted", 1);
  
    nh_.param("camera_wrt_laser_x", camera_wrt_laser_x_, (float)CAM_WRT_LASER_X);
    nh_.param("camera_wrt_laser_y", camera_wrt_laser_y_, (float)CAM_WRT_LASER_Y);
    nh_.param("camera_wrt_laser_z", camera_wrt_laser_z_, (float)CAM_WRT_LASER_Z);
    nh_.param("camera_wrt_laser_pitch", camera_wrt_laser_pitch_, (float)CAM_WRT_LASER_PITCH);
    nh_.param("camera_parent_frame", camera_parent_frame_,  std::string("laser"));
    nh_.param(
      "fisheye_config", fisheye_config_file_, 
      ros::package::getPath("person_follower") + "/config/fisheye_calib.txt"
    );

    get_ocam_model(&fisheye_model_, (char*)fisheye_config_file_.c_str());
    

  }

  void detectionCallback(const yolo2::ImageDetections::ConstPtr &detectionMsg, const sensor_msgs::Image::ConstPtr &imageMsg)
  {

    std::vector<cv::Point2f> undistortedPoints;
    std::vector<float> depthValues;
    std::vector<cv::Point2f> vectPersonPoints;
    
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(imageMsg);

    cv::Mat undistorted_image(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat undistorted_x(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat undistorted_y(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);

    /* --------------------------------------------------------------------  */  
    /* Create Look-Up-Table for perspective undistortion                     */
    /* SF is kind of distance from the undistorted image to the camera       */
    /* (it is not meters, it is justa zoom fator)                            */
    /* Try to change SF to see how it affects the result                     */
    /* The undistortion is done on a  plane perpendicular to the camera axis */
    /* --------------------------------------------------------------------  */
    float sf = 4;
    CvMat temp_x = undistorted_x, temp_y = undistorted_y;
    create_perspecive_undistortion_LUT( &temp_x, &temp_y, &fisheye_model_, sf );
    cv::remap( 
      cv_ptr->image.clone(), undistorted_image, undistorted_x, undistorted_y, 
      cv::INTER_CUBIC, cv::BORDER_CONSTANT,
      cv::Scalar(0) 
    );

    cv::Mat undistorted_image_8bit;
    undistorted_image.convertTo(undistorted_image_8bit, CV_8UC1);
    cv_bridge::CvImage cv_bridge_image;
    cv_bridge_image.image = undistorted_image_8bit;
    cv_bridge_image.encoding = "mono8";
    cv_bridge_image.header = imageMsg->header;
    pub_undistorted_image_.publish(cv_bridge_image.toImageMsg());

    // cv::Mat matPersonSegmented = cv::Mat::zeros(imageMsg->height, imageMsg->width, CV_8UC1);
    
    // for (int i=0; i<detectionMsg->detections.size(); i++)
    // {
    //   if (detectionMsg->detections[i].class_id != 0)
    //   {
    //     continue;
    //   }

    // }
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "fisheye_blob_publisher"); 
  ros::NodeHandle n("~");

  FisheyeBlobPublisher fisheye_blob_publisher(n);
  image_transport::ImageTransport image_transporter(n);

  message_filters::Subscriber<yolo2::ImageDetections> detectionSub(n, "/vision/yolo2/detections", 1);
  image_transport::SubscriberFilter imageSub(image_transporter, "/camera/fisheye/image_raw", 1);
  
  typedef message_filters::sync_policies::ApproximateTime<yolo2::ImageDetections, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(1000), detectionSub, imageSub);
  sync.registerCallback(boost::bind(&FisheyeBlobPublisher::detectionCallback, &fisheye_blob_publisher, _1, _2));

  ros::spin();
  return 0;
}