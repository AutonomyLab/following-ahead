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
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <ros/package.h>

#include "robot.hpp"
#include "utils.hpp"
#include "config.h"
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
public:
  enum tracking_status_t {
    PERSON_SELECTED,
    PERSON_UNDER_CONSIDERATION,
    LOST
  };

private:
  tracking_status_t tracking_status_;
  size_t num_seen_person_under_consideration_; 

  ros::Publisher pubPointCloud_;
  ros::Publisher pubRelativePose_;
  // TODO: publisher for seed
  ros::Publisher pubPositionMeasurementSeeds_;
  ros::Publisher pubFilteredLaser_;

  ros::Subscriber subLegDetections_;

  sensor_msgs::PointCloud::Ptr pointCloudMsg_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  cv::Point3f human_prev_pose_;
  float camera_wrt_laser_x_;
  float camera_wrt_laser_y_;
  float camera_wrt_laser_z_;
  float camera_wrt_laser_pitch_;
  double person_lost_timeout_;
  double last_update_time_;
  std::string camera_parent_frame_;
  std::string camera_frame_;
  std::string map_frame_;
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
      image_transport_(n),
      tracking_status_(tracking_status_t::LOST),
      num_seen_person_under_consideration_(0)
  {
    pubPointCloud_ =  nh_.advertise<sensor_msgs::PointCloud>("person_cloud", 1);
    pubRelativePose_ = nh_.advertise<geometry_msgs::TransformStamped>("/person_follower/groundtruth_pose", 1);
    pubPositionMeasurementSeeds_ = nh_.advertise<people_msgs::PositionMeasurement>("/people_tracker_filter", 1);
    pubFilteredLaser_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
    pub_undistorted_image_ = image_transport_.advertise("/camera/fisheye/undistorted", 1);

    subLegDetections_ = nh_.subscribe("/people_tracker_measurements", 1000, 
                                      &FisheyeBlobPublisher::legDetectionCallback, this);
   
    nh_.param("person_lost_timeout", person_lost_timeout_, (double)PERSON_LOST_TIMEOUT);
    nh_.param("camera_wrt_laser_x", camera_wrt_laser_x_, (float)CAM_WRT_LASER_X);
    nh_.param("camera_wrt_laser_y", camera_wrt_laser_y_, (float)CAM_WRT_LASER_Y);
    nh_.param("camera_wrt_laser_z", camera_wrt_laser_z_, (float)CAM_WRT_LASER_Z);
    nh_.param("camera_wrt_laser_pitch", camera_wrt_laser_pitch_, (float)CAM_WRT_LASER_PITCH);
    nh_.param("camera_parent_frame", camera_parent_frame_,  std::string("laser"));
    nh_.param("camera_frame", camera_frame_,  std::string("camera"));
    nh_.param("map_frame", map_frame_,  std::string("map"));
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

  void legDetectionCallback(const people_msgs::PositionMeasurementArray::ConstPtr &detectionsMsg)
  {
    if (!detectionsMsg->people.size())
    {
      return;
    }

    ros::Time send_time = ros::Time::now();

    int selected_person_idx = -1;
    float min_leg_dist = 100;

    if  (
          tracking_status_ == tracking_status_t::PERSON_SELECTED ||
          tracking_status_ == tracking_status_t::PERSON_UNDER_CONSIDERATION
        )
    {
       // min_leg_dist = MIN_SOMETHING; TODO
      for (size_t person_idx = 0; person_idx < detectionsMsg->people.size(); person_idx++)
      {  
        float leg_dist = sqrt(
          pow(human_prev_pose_.x - detectionsMsg->people[person_idx].pos.x, 2) +
          pow(human_prev_pose_.y - detectionsMsg->people[person_idx].pos.y, 2)
        );
        if  ( 
               leg_dist < min_leg_dist 
            )
        {
          if (tracking_status_ == tracking_status_t::PERSON_UNDER_CONSIDERATION)
          {
            num_seen_person_under_consideration_++;
            ROS_INFO("person under consideration %d", num_seen_person_under_consideration_);
            // TODO: softcode this number
            if (num_seen_person_under_consideration_ > 10)
            {
              tracking_status_ = tracking_status_t::PERSON_SELECTED;
              ROS_INFO("person selected");
            }
          }

          last_update_time_ = send_time.toSec();
          human_prev_pose_.x =  detectionsMsg->people[person_idx].pos.x;
          human_prev_pose_.y =  detectionsMsg->people[person_idx].pos.y;
          human_prev_pose_.z =  detectionsMsg->people[person_idx].pos.z;
          selected_person_idx = person_idx;
          min_leg_dist = leg_dist;
          continue;
        }
      }
    }
    
    if (selected_person_idx == -1)
    {
      // the transformation from map frame to laser frame
      tf::StampedTransform camera_T_map;
      try
      {
        
        tf_listener_.lookupTransform(camera_frame_, map_frame_,
                                    ros::Time(0), camera_T_map);
      }
      catch (tf::TransformException ex)
      {
          ROS_ERROR("TF error %s", ex.what());
          return;
      }
      //find the closest one
      for (size_t person_idx = 0; person_idx < detectionsMsg->people.size(); person_idx++)
      { 
        
        cv::Point3f map_pos(
          detectionsMsg->people[person_idx].pos.x,
          detectionsMsg->people[person_idx].pos.y,
          detectionsMsg->people[person_idx].pos.z
        );

        cv::Point3f camera_pos = transformPoint(camera_T_map, map_pos);
        
        float leg_dist = sqrt(
          pow(camera_pos.x, 2) +
          pow(camera_pos.y, 2) +
          pow(camera_pos.z, 2) 
        );

        ROS_INFO("%d leg dist: %f", person_idx, leg_dist);

        if (leg_dist < min_leg_dist)
        {
          selected_person_idx = person_idx;
          min_leg_dist = leg_dist;
        }
      }

      if (selected_person_idx != -1)
      {
        // we found the closest one
        tracking_status_ = tracking_status_t::PERSON_UNDER_CONSIDERATION;
        num_seen_person_under_consideration_ = 0;
        human_prev_pose_ = cv::Point3f(
          detectionsMsg->people[selected_person_idx].pos.x,
          detectionsMsg->people[selected_person_idx].pos.y,
          detectionsMsg->people[selected_person_idx].pos.z
        );
        ROS_INFO("Person is under consideration");
      } 
      else
      {
        // nothing found, but still try to track the person later
        if (fabs(send_time.toSec() - last_update_time_) > person_lost_timeout_)
        {
          num_seen_person_under_consideration_ = 0;
          tracking_status_ = tracking_status_t::LOST;
          ROS_INFO("PERSON LOST!!");
        }
        else
        {
          ROS_INFO("Waiting for person to return");
        }
      }
    }


    if (tracking_status_ != tracking_status_t::PERSON_SELECTED || selected_person_idx==-1)
    {
      return;
    }
    

    human_prev_pose_ = cv::Point3f(
      detectionsMsg->people[selected_person_idx].pos.x,
      detectionsMsg->people[selected_person_idx].pos.y,
      detectionsMsg->people[selected_person_idx].pos.z
    );

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = send_time;
    transform_stamped.header.frame_id = detectionsMsg->people[selected_person_idx].header.frame_id;
    transform_stamped.transform.translation.x = detectionsMsg->people[selected_person_idx].pos.x;
    transform_stamped.transform.translation.y = detectionsMsg->people[selected_person_idx].pos.y;
    transform_stamped.transform.translation.z = detectionsMsg->people[selected_person_idx].pos.z;

    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    pubRelativePose_.publish(transform_stamped);

    tf::StampedTransform relative_tf;
    relative_tf.child_frame_id_ = "relative_pose"; // source
    relative_tf.frame_id_ = detectionsMsg->people[selected_person_idx].header.frame_id; // target
    relative_tf.stamp_ = transform_stamped.header.stamp;

    relative_tf.setOrigin(tf::Vector3( 
      transform_stamped.transform.translation.x, 
      transform_stamped.transform.translation.y, 
      transform_stamped.transform.translation.z
    ));

    relative_tf.setRotation(tf::Quaternion(0, 0, 0, 1));

    tf_broadcaster_.sendTransform(relative_tf);

  }

  void yoloDetectionCallback(const yolo2::ImageDetections::ConstPtr &detectionMsg, const sensor_msgs::LaserScan::ConstPtr &laserMsg, const sensor_msgs::Image::ConstPtr &imageMsg)
  {
    if (detectionMsg == NULL || laserMsg == NULL || imageMsg == NULL)
    {
      return;
    }

    ros::Time send_time = ros::Time::now();
    
    camera_tf_.stamp_ = send_time;
    tf_broadcaster_.sendTransform(camera_tf_);
    
    if (!detectionMsg->detections.size())
    {
      // nothing found, but still try to track the person later
      if (fabs(send_time.toSec() - last_update_time_) > person_lost_timeout_)
      {
        num_seen_person_under_consideration_ = 0;
        tracking_status_ = tracking_status_t::LOST;
        ROS_INFO("PERSON LOST!!");
      }
      else
      {
        ROS_INFO("Waiting for person to return");
      }
    }

    sensor_msgs::LaserScan laserMsgFiltered = *laserMsg;

    // bearing angles of endpoints
    double end_bearing_angles[2];
    float average_depth = 0;
    
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(imageMsg);
    
    laserMsgFiltered.header.stamp = send_time;
    std::fill(laserMsgFiltered.ranges.begin(), laserMsgFiltered.ranges.end(), laserMsgFiltered.range_max);
    
    for (uint32_t detection_idx=0; detection_idx<detectionMsg->detections.size(); detection_idx++)
    {
      if (detectionMsg->detections[detection_idx].class_id != 0)
      {
        continue;
      }

      // endpoints of blob in the image
      uint32_t row = std::min(
        (uint32_t)std::max(
          (uint32_t)(detectionMsg->detections[detection_idx].roi.y_offset + detectionMsg->detections[detection_idx].roi.height/2), 
          (uint32_t)0
        ),
        (uint32_t)(cv_ptr->image.rows-1)
      );

      uint32_t end_cols[2];
      end_cols[0] = std::min(
        (uint32_t)std::max((uint32_t)detectionMsg->detections[detection_idx].roi.x_offset, (uint32_t)0),
        (uint32_t)(cv_ptr->image.cols-1)
      );
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

        // find the angle of with respect to the x axis of laser (-z of camera)
        end_bearing_angles[i] = atan2(
          laser_bearing.y,
          laser_bearing.x
        );
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
        laserMsgFiltered.ranges[i] = laserMsg->ranges[i];
      }
    }

    if (tracking_status_ != tracking_status_t::LOST)
    {
      people_msgs::PositionMeasurement measurement_seed;
      measurement_seed.header.stamp = send_time;
      measurement_seed.header.frame_id = map_frame_;
      measurement_seed.name = "person";
      measurement_seed.object_id = "0";
      measurement_seed.pos.x = human_prev_pose_.x;
      measurement_seed.pos.x = human_prev_pose_.y;
      measurement_seed.pos.x = human_prev_pose_.z; 
      pubPositionMeasurementSeeds_.publish(measurement_seed);
    }

    pubFilteredLaser_.publish(laserMsgFiltered);

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
  sync.registerCallback(boost::bind(&FisheyeBlobPublisher::yoloDetectionCallback, &fisheye_blob_publisher, _1, _2, _3));

  ros::spin();
  return 0;
}