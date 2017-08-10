#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <algorithm>
#include "linear_motion_model.hpp"
#include "config.h"
#include "utils.hpp"

int LinearMotionModel::updateWayPoint(  cv::Mat &map, float map_resolution,
                                        cv::Point object_point, cv::Point destination_point, float distance,
                                        cv::Point &object_point_out, cv::Point &destination_point_out, float &remaining_distance_out,
                                        cv::Mat &debug_map
                                      )
{
  debug_map = map.clone();
	
  // add perturbations to the destination points to get a number of rays
  std::vector<cv::Point> predictions;
  for (int i = -OBSTACLE_RAYCASTING_PERTURBATION; i <= OBSTACLE_RAYCASTING_PERTURBATION; i++)
  {
    for (int j = -OBSTACLE_RAYCASTING_PERTURBATION; j <= OBSTACLE_RAYCASTING_PERTURBATION; j++)
    {
      int x = destination_point.x + i;
      int y = destination_point.y + j;
      x = std::max( std::min(x, map.cols-1), 0);
      y = std::max( std::min(y, map.rows-1), 0);
      predictions.push_back(
        cv::Point(x, y)
      );
    }
  }

  std::vector<cv::Point> obstacles;
  bool is_obstacle = false;

  for (size_t prediction_idx = 0; prediction_idx < predictions.size(); prediction_idx++)
  {
    // raycasting from object_point to prediction
    cv::LineIterator line_iterator(map, object_point, predictions[prediction_idx]);
    cv::LineIterator it = line_iterator;
    
    cv::Point obstacle_coordinates;
    for(int i = 0; i < line_iterator.count; i++, ++it)
    {

      if (map.at<uint8_t>(it.pos()) == 255)
      {
        // there is an obstacle
        is_obstacle = true;
        obstacles.push_back(it.pos());
        break;
      }
      debug_map.at<uint8_t>(it.pos()) = 127;
    }
  }

  if (is_obstacle)
  {
    cv::Mat obstacle_image = cv::Mat::zeros(map.rows, map.cols, CV_8UC1);
    
    size_t obstacle_count = 0;
    for (size_t obstacle_idx =0; obstacle_idx < obstacles.size(); obstacle_idx++)
    {
      if (obstacle_image.at<uint8_t>(obstacles[obstacle_idx]) != 255)
      {
        obstacle_count++;
        obstacle_image.at<uint8_t>(obstacles[obstacle_idx]) = 255;
        debug_map.at<uint8_t>(obstacles[obstacle_idx]) = 0;
      } 
    }
    // map = obstacle_image.clone();
    
    // cv::Mat mask = cv::Mat::zeros(map.rows + 2, map.cols + 2, CV_8UC1); 
    // cv::Rect bounding_box;
    // cv::floodFill(
    //   map, mask, obstacle_coordinates, 
    //   255, &bounding_box, 
    //   cv::Scalar(), cv::Scalar(),  4 | ((int)255 << 8) | cv::FLOODFILL_MASK_ONLY
    // );

    // std::cout << "bounding box: \n" << bounding_box << std::endl; 

    // the mask is padded by two pixels
    // map = mask.rowRange(1, mask.rows-1).colRange(1, mask.cols-1);

    // TODO: floodfill before doing the hough transform
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(obstacle_image, lines, 2, CV_PI/180.0*2.0, obstacle_count*0.7, 0, 0);

    // vector<Vec4i> lines;
    // HoughLinesP(obstacle_image, lines, 1, CV_PI/180, 15, 3, 1 );

    if (lines.size())
    {
      for (size_t line_idx = 0; line_idx < 1/*lines.size()*/; line_idx++)
      {
        // std::cout << "Line: " << lines[line_idx] << std::endl;
        
        float rho = lines[line_idx][0];
        float theta = lines[line_idx][1];

        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b)); //??
        pt1.y = cvRound(y0 + 1000*(a)); //??
        pt2.x = cvRound(x0 - 1000*(-b)); //??
        pt2.y = cvRound(y0 - 1000*(a)); //??
        cv::line(debug_map, pt1, pt2, cv::Scalar(150), 1, CV_AA);

        // find the new waypoint of the person
        // angle of the vector RP (Robot to Prediction)
        float RP_theta = atan2(
          destination_point.y - object_point.y,
          destination_point.x - object_point.x
        );

        float clearance = OBSTACLE_CLEARANCE_DISTANCE / map_resolution;
        object_point_out.x = round(obstacles[0].x - clearance * cos(RP_theta));
        object_point_out.y = round(obstacles[0].y - clearance * sin(RP_theta));
        object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
        object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);

        // find the waypoint of prediction
        // angle of the obstacle line
        float ol_theta = atan2(
          -cos(theta),
          sin(theta)
        );
        ol_theta = chooseObstacleDirection(RP_theta, ol_theta);

        // move remaining distance away from the robot waypoint towards the obstacle direction (wall following)
        // TODO: CALCULATE THE REMAINING DISTANCE!!!
        
        destination_point_out.x = object_point_out.x + 100 * cos(ol_theta);
        destination_point_out.y = object_point_out.y + 100 * sin(ol_theta); 
        destination_point_out.x = std::max( std::min(destination_point_out.x, map.cols-1), 0);
        destination_point_out.y = std::max( std::min(destination_point_out.y, map.rows-1), 0);

        // visualize the new waypoints
        cv::line(debug_map, object_point_out, destination_point_out, cv::Scalar(150), 2, CV_AA);
      }
    }
    else
    {
      // no line found even if there are obstacles
      // TODO: deal with this more gracefully
      ROS_WARN("Could not find obstacle line, consider changing hough threshold");
      return 1;  
    }
  }

  // visualize the robot and the person
  cv::circle(debug_map, object_point, 8, 255);
  cv::circle(debug_map, destination_point, 5, 255);

  return 0;

}

float LinearMotionModel::chooseObstacleDirection(float RP_theta, float ol_theta)
{
  float ol_theta2 = oppositeAngle(ol_theta);

  cv::Mat R_w_rp = theta2RotationMatrix(RP_theta);
  cv::Mat R_w_vl1 = theta2RotationMatrix(ol_theta);
  cv::Mat R_w_vl2 = theta2RotationMatrix(ol_theta2);

  cv::Mat R_rp_vl1 = R_w_rp.t() * R_w_vl1;
  cv::Mat R_rp_vl2 = R_w_rp.t() * R_w_vl2;

  // angles between the vectors RP and OL
  float angle1 = rotationMatrix2Theta(R_rp_vl1);
  float angle2 = rotationMatrix2Theta(R_rp_vl2);

  return (abs(angle2) < abs(angle1)) ? ol_theta2 : ol_theta;

}

float LinearMotionModel::oppositeAngle(float angle)
{
  // find the direction of the unit vector in the opposite direction
  return atan2(-sin(angle), -cos(angle));
}