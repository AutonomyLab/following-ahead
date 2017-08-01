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

#include "pid.h"

// void detectionCallback(const yolo2::ImageDetections::ConstPtr &detectionMsg, const sensor_msgs::Image::ConstPtr &depthMsg)
// {
//   std::vector<cv::Point2f> distortedPoints;
//   std::vector<cv::Point2f> undistortedPoints;

//   cv_bridge::CvImageConstPtr cv_ptr;
//   cv_ptr = cv_bridge::toCvShare(depthMsg);
  
//   std::vector<cv::Point2f> vectPersonPoints;

//   cv::Mat matPersonSegmented = cv::Mat::zeros(depthMsg->height, depthMsg->width, CV_8UC1);
//   for (int i=0; i<detectionMsg->detections.size(); i++)
//   {
//     if (detectionMsg->detections[i].class_id != 0)
//       continue;
//     // Stg::ModelBlobfinder  myFinder;
//     // std::cout << detectionMsg->detections[i].class_id << std::endl;
//     // std::cout << detectionMsg->detections[i].x << ", " << detectionMsg->detections[i].y << std::endl;
//     // std::cout << detectionMsg->detections[i].width << ", " << detectionMsg->detections[i].height << std::endl << std::endl  ;
//     // // myFinder.fov = 70.0*M_PI/180.0;
//     // myFinder.
//     distortedPoints.push_back(cv::Point2f(
//       detectionMsg->detections[i].roi.x_offset + detectionMsg->detections[i].roi.width/2.0 ,
//       detectionMsg->detections[i].roi.y_offset + detectionMsg->detections[i].roi.height/2.0

//     ));

//     std::vector<float> depthValues;

//     for (
//           int row=detectionMsg->detections[i].roi.y_offset; 
//           row<detectionMsg->detections[i].roi.y_offset+detectionMsg->detections[i].roi.height;
//           row++
//         )
//     {
//       for (
//             int col=detectionMsg->detections[i].roi.x_offset; 
//             col<detectionMsg->detections[i].roi.x_offset+detectionMsg->detections[i].roi.width;
//             col++
//           )
//       {
//         vectPersonPoints.push_back(cv::Point2f(col, row));
//         depthValues.push_back(cv_ptr->image.at<float>(row, col));
//       }
//     }

//     // float medianDepth = median(depthValues);

//     // for (
//     //       int row=detectionMsg->detections[i].roi.y_offset; 
//     //       row<detectionMsg->detections[i].roi.y_offset+detectionMsg->detections[i].roi.height;
//     //       row++
//     //     )
//     // {
//     //   for (
//     //         int col=detectionMsg->detections[i].roi.x_offset; 
//     //         col<detectionMsg->detections[i].roi.x_offset+detectionMsg->detections[i].roi.width;
//     //         col++
//     //       )
//     //   {
//     //     if (
//     //           cv_ptr->image.at<float>(row, col) >= medianDepth-0.12 &&
//     //           cv_ptr->image.at<float>(row, col) <= medianDepth+0.12
//     //       )   
//     //     {
//     //       matPersonSegmented.at<uint8_t>(row, col) = 255;
//     //     }
        
//     //   }
//     // }



//   }

//   // cv::imshow("window", matPersonSegmented);
//   // cv::waitKey(5);

//   if (distortedPoints.size()==0)
//   {
//     return;
//   }

//   cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
//   cameraMatrix.at<float>(0, 0) = focalLengthX;
//   cameraMatrix.at<float>(1, 1) = focalLengthY;
//   cameraMatrix.at<float>(0, 2) = cameraPrincipalX;
//   cameraMatrix.at<float>(1, 2) = cameraPrincipalY;

//   cv::Mat newCameraMatrix(3, 3, CV_32F);
  
//   cv::Vec<float, 5> distortionCoeffs;
//   distortionCoeffs[0] = 0.176206;
//   distortionCoeffs[1] = -0.363409;
//   distortionCoeffs[2] = 0.001018;
//   distortionCoeffs[3] =  -0.003290;
//   distortionCoeffs[4] = 0.000000;

//   cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix, distortionCoeffs);

//   std::cout << "Depth image size: " <<  cv_ptr->image.size() << std::endl;
//   for (size_t i = 0; i < undistortedPoints.size(); i++)
//   {
//     cv::Point2f pt = undistortedPoints[i];
//     float depth = cv_ptr->image.at<float>( (size_t)distortedPoints[i].y, (size_t)distortedPoints[i].x );

//     float x = (distortedPoints[i].x - cameraPrincipalX)/focalLengthX*depth;
//     float y = (distortedPoints[i].y - cameraPrincipalY)/focalLengthY*depth;
//     float z = depth;

//     std::cout << distortedPoints[i] << std::endl;
//     std::cout << "(" 
//               << x << ", "
//               << y << ", "
//               << z << ") "
//               << std::endl
//               << std::endl;

//     // std::cout << getBlobBearing(pt.x * cameraMatrix.at<float>(0, 0)) * 180.0 / M_PI << std::endl;
//   }

//   undistortedPoints.clear();
//   cv::undistortPoints(vectPersonPoints, undistortedPoints, cameraMatrix, distortionCoeffs);
//   pointCloudMsg->header.frame_id = "world";
//   // pointCloudMsg->height = pointCloudMsg->width = 1;
//   pointCloudMsg->header.stamp = ros::Time::now();
//   pointCloudMsg->points.clear();

//   for (size_t i = 0; i < undistortedPoints.size(); i++)
//   {
//     cv::Point2f pt = undistortedPoints[i];
//     float depth = cv_ptr->image.at<float>( (size_t)vectPersonPoints[i].y, (size_t)vectPersonPoints[i].x );

//     float x = (vectPersonPoints[i].x - cameraPrincipalX)/focalLengthX*depth;
//     float y = (vectPersonPoints[i].y - cameraPrincipalY)/focalLengthY*depth;
//     float z = depth;

//     geometry_msgs::Point32 point;
//     point.x = -x;
//     point.y = -z;
//     point.z = -y;

//     pointCloudMsg->points.push_back(point);
//   }


//   pubPointCloud.publish(pointCloudMsg);
  
  
//   std::cout << std::endl;
// }


int main(int argc, char** argv )
{

  ros::Publisher pubPointCloud;
  sensor_msgs::PointCloud::Ptr pointCloudMsg (new sensor_msgs::PointCloud);

  ros::init(argc, argv, "person_follower_node"); 
  ros::NodeHandle n("~");

  std::string base_frame;
  std::string odom_frame;
  std::string map_frame;
  std::string person_frame;

  if (!n.getParam("base_frame", base_frame))
  {
    ROS_WARN("no param for base_frame, using default");
    base_frame = "base_frame";
  }

  if (!n.getParam("odom_frame", odom_frame))
  {
    ROS_WARN("no param for odom_frame, using default");
    odom_frame = "odom_frame";
  }

  if (!n.getParam("map_frame", map_frame))
  {
    ROS_WARN("no param for map_frame, using default");
    map_frame = "map_frame";
  }

  if (!n.getParam("person_frame", person_frame))
  {
    ROS_WARN("no param for person_frame, using default");
    person_frame = "person_frame";
  }

  
  Robot myRobot(
    n, base_frame, odom_frame, map_frame, person_frame
  );

  // message_filters::Subscriber<yolo2::ImageDetections> detectionSub(n, "vision/yolo2/detections", 1);
  // message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "camera/depth_registered/sw_registered/image_rect", 1);
  // typedef message_filters::sync_policies::ApproximateTime<yolo2::ImageDetections, sensor_msgs::Image> SyncPolicy;
  // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), detectionSub, depthSub);
  // sync.registerCallback(boost::bind(&detectionCallback, _1, _2));
  // ros::Subscriber detection_sub = n.subscribe("/vision/yolo2/detections", 10, detectionCallback);

  pubPointCloud =  n.advertise<sensor_msgs::PointCloud>("person_cloud", 1);
  
  ros::Subscriber odo_sub = n.subscribe("/husky/odom", 1, &Robot::odometryCallback, &myRobot);
  ros::Subscriber joy_sub = n.subscribe("/teleop/joy", 1, &Robot::joyCallback, &myRobot);  
  ros::Subscriber groundtruth_sub = n.subscribe("/person_follower/groundtruth_pose", 1, &Robot::myBlobUpdate, &myRobot);
  

  ros::spin();




  return 0; // ok
}

// Stg::meters_t getRangeFromLaser(robot_t *robot, double startAngle, double endAngle)
// {
//   // get the data
//   std::vector<Stg::meters_t> scan = robot->laser->GetSensors()[0].ranges;
//   // make the orientation of laser scan counter-clockwise (+ve) to the robot
//   std::reverse(scan.begin(), scan.end());

//   double angleIncrement = robot->laser->GetSensors()[0].fov/(double)(robot->laser->GetSensors()[0].sample_count-1);
//   size_t startIdx = round(startAngle/angleIncrement);
//   size_t endIdx = round(endAngle/angleIncrement);

//   // debug
//   // std::cout << "Non saturated laser" << std::endl;
//   // for (size_t i=0; i<scan.size(); i++)
//   // {
//   //   if (scan[i]<8)
//   //   {
//   //     std::cout << i << " ";
//   //   }
//   // }
//   // std::cout << std::endl;

//   // std::cout << "Start: " << startIdx << " End: " << endIdx << std::endl;
  
//   std::vector<Stg::meters_t> validScan;
//   for (size_t i=startIdx; i!=endIdx+1; i++)
//   {
//     i = i % robot->laser->GetSensors()[0].sample_count;
//     if (scan[i] < robot->laser->GetSensors()[0].range.max)
//     {
//       // std::cout << scan[i] << std::endl;
//       validScan.push_back(scan[i]);
//     }
//   }
  
//   if (validScan.size() == 0)
//   {
//     std::cout << "No valid scan" << std::endl;
//     return 0;
//   }

//   // median
//   size_t medianIdx = validScan.size()/2;
//   std::nth_element(validScan.begin(), validScan.begin()+medianIdx, validScan.end());
//   return validScan[medianIdx];
//   // min
//   // return *(std::min_element(validScan.begin(), validScan.end()));

// }



