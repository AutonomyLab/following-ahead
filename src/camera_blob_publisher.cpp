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

class CameraBlobPublisher
{
private:
  ros::Publisher pubPointCloud_;
  sensor_msgs::PointCloud::Ptr pointCloudMsg_;
  ros::NodeHandle nh_;

public:
  CameraBlobPublisher(ros::NodeHandle n)
    : nh_(n),
      pointCloudMsg_(new sensor_msgs::PointCloud)
  {
    pubPointCloud_ =  nh_.advertise<sensor_msgs::PointCloud>("person_cloud", 1);
  }

  void detectionCallback(const yolo2::ImageDetections::ConstPtr &detectionMsg, const sensor_msgs::Image::ConstPtr &depthMsg)
  {
    std::vector<cv::Point2f> distortedPoints;
    std::vector<cv::Point2f> undistortedPoints;

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(depthMsg);
    
    std::vector<cv::Point2f> vectPersonPoints;
    cv::Mat matPersonSegmented = cv::Mat::zeros(depthMsg->height, depthMsg->width, CV_8UC1);
    
    for (int i=0; i<detectionMsg->detections.size(); i++)
    {
      if (detectionMsg->detections[i].class_id != 0)
        continue;
      
      // Stg::ModelBlobfinder  myFinder;
      // std::cout << detectionMsg->detections[i].class_id << std::endl;
      // std::cout << detectionMsg->detections[i].x << ", " << detectionMsg->detections[i].y << std::endl;
      // std::cout << detectionMsg->detections[i].width << ", " << detectionMsg->detections[i].height << std::endl << std::endl  ;
      // // myFinder.fov = 70.0*M_PI/180.0;
      // myFinder.
      
      distortedPoints.push_back(cv::Point2f(
        detectionMsg->detections[i].roi.x_offset + detectionMsg->detections[i].roi.width/2.0 ,
        detectionMsg->detections[i].roi.y_offset + detectionMsg->detections[i].roi.height/2.0

      ));

      std::vector<float> depthValues;

      for (
            int row=detectionMsg->detections[i].roi.y_offset; 
            row<detectionMsg->detections[i].roi.y_offset+detectionMsg->detections[i].roi.height;
            row++
          )
      {
        for (
              int col=detectionMsg->detections[i].roi.x_offset; 
              col<detectionMsg->detections[i].roi.x_offset+detectionMsg->detections[i].roi.width;
              col++
            )
        {
          float depth = cv_ptr->image.at<float>(row, col);
          if (depth < 0.01 || depth > 6.0 || !std::isfinite(depth))
          {
            continue;
          }

          vectPersonPoints.push_back(cv::Point2f(col, row));
          depthValues.push_back(depth);
        }
      }

      size_t medianIdx = depthValues.size()/2;
      std::nth_element(depthValues.begin(), depthValues.begin()+medianIdx, depthValues.end());
      float medianDepth = depthValues[medianIdx];

      std::vector<cv::Point2f> vectPersonPointsSegmented;

      for (size_t i = 0; i < vectPersonPoints.size(); i++)        
      {
        int row = vectPersonPoints[i].y;
        int col = vectPersonPoints[i].x;

        if (
              cv_ptr->image.at<float>(row, col) >= medianDepth-0.12 &&
              cv_ptr->image.at<float>(row, col) <= medianDepth+0.12
            )   
        {
          vectPersonPointsSegmented.push_back(vectPersonPoints[i]);
        }
      }

      vectPersonPoints = vectPersonPointsSegmented;
    }

    // cv::imshow("window", matPersonSegmented);
    // cv::waitKey(5);

    if (distortedPoints.size()==0)
    {
      return;
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
    cameraMatrix.at<float>(0, 0) = focalLengthX;
    cameraMatrix.at<float>(1, 1) = focalLengthY;
    cameraMatrix.at<float>(0, 2) = cameraPrincipalX;
    cameraMatrix.at<float>(1, 2) = cameraPrincipalY;

    cv::Mat newCameraMatrix(3, 3, CV_32F);
    
    cv::Vec<float, 5> distortionCoeffs;
    
    distortionCoeffs[0] = 0.07346920669078827;
    distortionCoeffs[1] = -0.12585918605327606;
    distortionCoeffs[2] = -0.0005786623223684728;
    distortionCoeffs[3] = -0.0006174440495669842;
    distortionCoeffs[4] = 0.000000;

    cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix, distortionCoeffs);

    std::cout << "Depth image size: " <<  cv_ptr->image.size() << std::endl;
    std::cout << "undistortedPoints: " << undistortedPoints.size() << std::endl;
    for (size_t i = 0; i < undistortedPoints.size(); i++)
    {
      cv::Point2f pt = undistortedPoints[i];
      float depth = cv_ptr->image.at<float>( (size_t)distortedPoints[i].y, (size_t)distortedPoints[i].x );

      float x = (distortedPoints[i].x - cameraPrincipalX)/focalLengthX*depth;
      float y = (distortedPoints[i].y - cameraPrincipalY)/focalLengthY*depth;
      float z = depth;

      std::cout << distortedPoints[i] << std::endl;
      std::cout << "(" 
                << x << ", "
                << y << ", "
                << z << ") "
                << std::endl
                << std::endl;

      std::cout << "depth: " << z << std::endl;

      // std::cout << getBlobBearing(pt.x * cameraMatrix.at<float>(0, 0)) * 180.0 / M_PI << std::endl;
    }

    undistortedPoints.clear();
    cv::undistortPoints(vectPersonPoints, undistortedPoints, cameraMatrix, distortionCoeffs);
    pointCloudMsg_->header.frame_id = "world";
    // pointCloudMsg_->height = pointCloudMsg_->width = 1;
    pointCloudMsg_->header.stamp = ros::Time::now();
    pointCloudMsg_->points.clear();

    for (size_t i = 0; i < undistortedPoints.size(); i++)
    {
      cv::Point2f pt = undistortedPoints[i];
      float depth = cv_ptr->image.at<float>( (size_t)vectPersonPoints[i].y, (size_t)vectPersonPoints[i].x );

      float x = (vectPersonPoints[i].x - cameraPrincipalX)/focalLengthX*depth;
      float y = (vectPersonPoints[i].y - cameraPrincipalY)/focalLengthY*depth;
      float z = depth;

      geometry_msgs::Point32 point;
      point.x = -x;
      point.y = -z;
      point.z = -y;

      pointCloudMsg_->points.push_back(point);
    }


    pubPointCloud_.publish(pointCloudMsg_);
    
    
    std::cout << std::endl;
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "camera_blob_publisher"); 
  ros::NodeHandle n("~");

  CameraBlobPublisher camera_blob_publisher(n);

  message_filters::Subscriber<yolo2::ImageDetections> detectionSub(n, "/vision/yolo2/detections", 1);
  message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "/camera/depth_registered/sw_registered/image_rect", 1);
  typedef message_filters::sync_policies::ApproximateTime<yolo2::ImageDetections, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), detectionSub, depthSub);
  sync.registerCallback(boost::bind(&CameraBlobPublisher::detectionCallback, &camera_blob_publisher, _1, _2));

  ros::spin();
  return 0;
}