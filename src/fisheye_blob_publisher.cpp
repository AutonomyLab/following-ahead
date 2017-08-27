#define CAM_WRT_LASER_X -0.03
#define CAM_WRT_LASER_Y 0.0
#define CAM_WRT_LASER_Z 0.094

#define CAM_WRT_LASER_PITCH 5

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
#include <sensor_msgs/LaserScan.h>
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
#include <numeric>
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
  std::string camera_frame_;
  std::string fisheye_config_file_;

  tf::StampedTransform camera_tf_;

  image_transport::ImageTransport image_transport_;
  image_transport::Publisher pub_undistorted_image_;

  struct ocam_model fisheye_model_;

public:

  void computeCameraTF()
  {
    // ----------------------- broadcast camera tf wrt world ---------------------------------
    camera_tf_.child_frame_id_ = camera_frame_; // source
    camera_tf_.frame_id_ = camera_parent_frame_; // target
    camera_tf_.stamp_ = ros::Time::now();

    camera_tf_.setOrigin(tf::Vector3( 
      camera_wrt_laser_x_, 
      camera_wrt_laser_y_, 
      camera_wrt_laser_z_
    )); 

    float pitch_angle = camera_wrt_laser_pitch_ * M_PI / 180.;
    
    cv::Point3f basis_y(
      0, -1, 0
    );

    cv::Point3f basis_z(
      -cos(pitch_angle),
      0,
      -sin(pitch_angle)
    );

    cv::Point3f basis_x = basis_y.cross(basis_z);
    
    tf::Quaternion q;
    tf::Matrix3x3 rotation(
      basis_x.x, basis_y.x, basis_z.x,
      basis_x.y, basis_y.y, basis_z.y,
      basis_x.z, basis_y.z, basis_z.z 
    );
    rotation.getRotation(q);
    camera_tf_.setRotation(q);

  }

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
    nh_.param("camera_frame", camera_frame_,  std::string("camera"));
    nh_.param(
      "fisheye_config", fisheye_config_file_, 
      ros::package::getPath("person_follower") + "/config/fisheye_calib.txt"
    );

    get_ocam_model(&fisheye_model_, (char*)fisheye_config_file_.c_str());
    computeCameraTF();

  }

  void undistortImage(cv_bridge::CvImageConstPtr cv_ptr, cv::Mat &undistorted_image_8bit)
  {
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

    undistorted_image.convertTo(undistorted_image_8bit, CV_8UC1);
  }

  void detectionCallback(const yolo2::ImageDetections::ConstPtr &detectionMsg, const sensor_msgs::LaserScan::ConstPtr &laserMsg, const sensor_msgs::Image::ConstPtr &imageMsg)
  {
    if (detectionMsg == NULL || laserMsg == NULL || imageMsg == NULL)
    {
      return;
    }
    ROS_INFO("callback");
    std::vector<cv::Point2f> undistortedPoints;
    std::vector<float> depthValues;
    std::vector<cv::Point2f> vectPersonPoints;

    // bearing angles of endpoints
    double end_bearing_angles[2];
    float average_depth = 0;
    
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(imageMsg);
    ROS_INFO("IMAGE");

    for (uint32_t detection_idx=0; detection_idx<detectionMsg->detections.size(); detection_idx++)
    {
      ROS_INFO("LOOP");
      if (detectionMsg->detections[detection_idx].class_id != 0)
      {
        continue;
      }

      std::cout << "row" << std::endl;
      // endpoints of blob in the image
      uint32_t row = std::min(
        (uint32_t)std::max(
          (uint32_t)(detectionMsg->detections[detection_idx].roi.y_offset + detectionMsg->detections[detection_idx].roi.height/2), 
          (uint32_t)0
        ),
        (uint32_t)(cv_ptr->image.rows-1)
      );

      uint32_t end_cols[2];
      std::cout << "col1" << std::endl;
      end_cols[0] = std::min(
        (uint32_t)std::max((uint32_t)detectionMsg->detections[detection_idx].roi.x_offset, (uint32_t)0),
        (uint32_t)(cv_ptr->image.cols-1)
      );
      std::cout << "col2" << std::endl;
      end_cols[1] = std::min(
        (uint32_t)std::max(
          (uint32_t)(detectionMsg->detections[detection_idx].roi.x_offset + detectionMsg->detections[detection_idx].roi.width), 
          (uint32_t)0
        ),
        (uint32_t)(cv_ptr->image.cols-1)
      );
      
      computeCameraTF();
      for (size_t i = 0; i < 2; i++)
      {
        double point2D[2] = {
          (double)row,
          (double)end_cols[i]
        };
        double point3D[3];
        cam2world(point3D, point2D, &fisheye_model_);

        tf::Transform R_laser_camera;
        R_laser_camera.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        R_laser_camera.setRotation(camera_tf_.getRotation());

        cv::Point3f camera_bearing(0, point3D[1], point3D[2]);
        // the rotation doesn't bloody work!!!!
        // cv::Point3f laser_bearing = transformPoint(R_laser_camera, camera_bearing);
        cv::Point3f laser_bearing(-point3D[2], -point3D[1], 0);

        std::cout << "point: " << point3D[0] << ", " << point3D[1] << ", " << point3D[2] << std::endl;
        std::cout << "point: " << laser_bearing.x << ", " << laser_bearing.y << ", " << laser_bearing.z << std::endl;

        // find the angle of with respect to the x axis of laser (-z of camera)
        end_bearing_angles[i] = atan2(
          laser_bearing.y,
          laser_bearing.x
        );

        // end_bearing_angles[i] = atan2(
        //   point3D[1],
        //   point3D[2]
        // );

        // std::cout << "camera angle: " << atan2(
        //   point3D[1],
        //   point3D[2]
        // ) * 180 / M_PI;

        // std::cout << "camera bearing: " << camera_bearing << std::endl;
        // std::cout << "laser bearing: " << laser_bearing << std::endl;

        std::cout << "bearing angle: " << end_bearing_angles[i] * 180 / M_PI << std::endl;
      }
      
      double min_bearing_angle = std::min(end_bearing_angles[0], end_bearing_angles[1]);
      double max_bearing_angle = std::max(end_bearing_angles[0], end_bearing_angles[1]);
      end_bearing_angles[0] = min_bearing_angle;
      end_bearing_angles[1] = max_bearing_angle;

      size_t laser_start_idx = std::min(
        (size_t)std::max(
          (size_t)round( (min_bearing_angle - laserMsg->angle_min)/laserMsg->angle_increment ),
          (size_t)0
        ),
        (size_t)(laserMsg->ranges.size() - 1)
      );
      size_t laser_end_idx = std::min(
        (size_t)(size_t)std::max(
          (size_t)round( (max_bearing_angle - laserMsg->angle_min)/laserMsg->angle_increment ),
          (size_t)0
        ),
        (size_t)(laserMsg->ranges.size() - 1)
      );

      // check depth based on laser
      for (size_t i = laser_start_idx; i < laser_end_idx; i++)
      {
        float depth = laserMsg->ranges[i];
        if (depth < 0.01 || depth > 3.5 || !std::isfinite(depth))
        {
          continue;
        }

        depthValues.push_back(depth);
      }

      if (depthValues.size() == 0)
      {
        ROS_WARN("No point in person: %d", detection_idx);
        continue;
      }

      size_t medianIdx = depthValues.size()/2;
      std::nth_element(depthValues.begin(), depthValues.begin()+medianIdx, depthValues.end());
      float medianDepth = depthValues[medianIdx];

      std::vector<float> depthValuesSegmented;

      for (size_t i = 0; i < depthValues.size(); i++)        
      {
        float depth = depthValues[i];
        if (
              depth >= medianDepth-0.12 &&
              depth <= medianDepth+0.12
            )   
        {
          depthValuesSegmented.push_back(depthValues[i]);
          average_depth += depthValues[i];
        }
      }
      depthValues = depthValuesSegmented;
      average_depth /= (float)depthValues.size();

      break;
    }

    if (depthValues.size() == 0)
    {
      ROS_WARN("No person");
      return;
    }
    
    // reduce the FOV of blob
    double average_bearing = (end_bearing_angles[0] + end_bearing_angles[1])/2.0;
    double range_bearing = (end_bearing_angles[1] - end_bearing_angles[0])/4.0;

    end_bearing_angles[0] = average_bearing - range_bearing;
    end_bearing_angles[1] = average_bearing + range_bearing;

    size_t medianIdx = depthValues.size()/2;
    std::nth_element(depthValues.begin(), depthValues.begin()+medianIdx, depthValues.end());
    average_depth = depthValues[medianIdx];

    ros::Time send_time = ros::Time::now();

    ROS_INFO(
      "bearing: %f, %f\n", 
      end_bearing_angles[0] * 180 / M_PI,
      end_bearing_angles[1] * 180 / M_PI
    );

    pointCloudMsg_->header.frame_id = camera_parent_frame_; //camera_frame_;
    pointCloudMsg_->header.stamp = send_time;
    pointCloudMsg_->points.clear();
    for (double bearing_angle = end_bearing_angles[0]; bearing_angle <= end_bearing_angles[1]; bearing_angle += 0.01)
    {
      geometry_msgs::Point32 point;
      point.x = cos(bearing_angle) * average_depth;
      point.y = sin(bearing_angle) * average_depth;
      point.z = 0;

      // point.z = cos(bearing_angle) * average_depth;
      // point.y = sin(bearing_angle) * average_depth;
      // point.x = 0;

      pointCloudMsg_->points.push_back(point);
    }
    ROS_INFO("point cloud create");
    
    // for (double *bearing_vector: bearing_vectors)
    // {
    //   geometry_msgs::Point32 point;
      
    //   double scale = 2;
    //   point.x = bearing_vector[0] * scale;
    //   point.y = bearing_vector[1] * scale;
    //   point.z = bearing_vector[2] * scale;

    //   pointCloudMsg_->points.push_back(point);
    // }
    pubPointCloud_.publish(pointCloudMsg_);
    ROS_INFO("point cloud pub");
    camera_tf_.stamp_ = send_time;
    tf_broadcaster_.sendTransform(camera_tf_);
    
    ROS_INFO("tf pub");
    // cv::Mat undistorted_image_8bit;
    // undistortImage(cv_ptr, undistorted_image_8bit);
    // cv_bridge::CvImage cv_bridge_image;
    // cv_bridge_image.image = undistorted_image_8bit;
    // cv_bridge_image.encoding = "mono8";
    // cv_bridge_image.header = imageMsg->header;
    // pub_undistorted_image_.publish(cv_bridge_image.toImageMsg());

  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "fisheye_blob_publisher"); 
  ros::NodeHandle n("~");

  FisheyeBlobPublisher fisheye_blob_publisher(n);
  image_transport::ImageTransport image_transporter(n);

  message_filters::Subscriber<yolo2::ImageDetections> detectionSub(n, "/vision/yolo2/detections", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> laserSub(n, "/scan", 1);
  image_transport::SubscriberFilter imageSub(image_transporter, "/camera/fisheye/image_raw", 1);
  
  typedef message_filters::sync_policies::ApproximateTime<yolo2::ImageDetections, sensor_msgs::LaserScan, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(1000), detectionSub, laserSub, imageSub);
  sync.registerCallback(boost::bind(&FisheyeBlobPublisher::detectionCallback, &fisheye_blob_publisher, _1, _2, _3));

  ros::spin();
  return 0;
}