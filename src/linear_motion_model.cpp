#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <algorithm>
#include <queue>
#include <map>
#include "linear_motion_model.hpp"
#include "config.h"
#include "utils.hpp"

LinearMotionModel::LinearMotionModel(): is_first_(true)
{

}

bool LinearMotionModel::checkObstacleBetween(cv::Point point1, cv::Point point2, cv::Mat &map, cv::Mat *debug_map)
{
  // raycasting from object_point to prediction
  cv::LineIterator line_iterator(map, point1, point2);
  cv::LineIterator it = line_iterator;
  
  bool is_obstacle = false;
  for(int i = 0; i < line_iterator.count; i++, ++it)
  {
    if (debug_map && debug_map->total())
    {
      (*debug_map).at<uint8_t>(it.pos()) = 255;
    }

    if (map.at<uint8_t>(it.pos()) == 255)
    {
      // there is an obstacle
      is_obstacle = true;
      break;
    }
  }
  return is_obstacle;
}

void LinearMotionModel::getViablDirections(
  cv::Point object_point,
  float obstacle_orientations[2],
  cv::Mat &map,
  float map_resolution,
  std::vector<size_t> &viable_obstacle_indices
)
{
  for (size_t obstacle_idx = 0; obstacle_idx < 2; obstacle_idx++)
  {
    // check if going along that direction leads to a dead end
    cv::Point goal_point(
      round((float)object_point.x + cos(obstacle_orientations[obstacle_idx]) * DEADEND_LOOKAHEAD_DISTANCE / map_resolution),
      round((float)object_point.y + sin(obstacle_orientations[obstacle_idx]) * DEADEND_LOOKAHEAD_DISTANCE / map_resolution)
    );
    goal_point.x = std::max( std::min(goal_point.x, map.cols-1), 0);
    goal_point.y = std::max( std::min(goal_point.y, map.rows-1), 0);

    // std::cout << "Checking goal: " << goal_point << std::endl;

    cv::Point new_destination_point;
    cv::Mat debug_map;
    bool is_feasible = LinearMotionModel::checkObjectDestinationFeasibility(
      object_point,
      goal_point,
      map,
      map_resolution,
      new_destination_point,
      debug_map
    );

    if (is_feasible)
    {
      viable_obstacle_indices.push_back(obstacle_idx);
    }
  }

}

bool LinearMotionModel::checkObjectDestinationFeasibility(
    cv::Point object_point, 
    cv::Point destination_point, 
    cv::Mat &map,
    float map_resolution,
    cv::Point &new_destination_point,
    cv::Mat &debug_map
)
{
  cv::Point2f object_destination_vector(
    destination_point.x - object_point.x,
    destination_point.y - object_point.y
  );

  float object_destination_norm = cv::norm(object_destination_vector);
  cv::Point2f object_destination_unit_vector = object_destination_vector / object_destination_norm;
  float new_object_destination_norm = object_destination_norm + FEASIBLE_DESTINATION_TO_OBSTACLE_DISTANCE/map_resolution;
  
  new_destination_point.x = round(object_point.x + new_object_destination_norm * object_destination_unit_vector.x);
  new_destination_point.y = round(object_point.y + new_object_destination_norm * object_destination_unit_vector.y);
  new_destination_point.x = std::max( std::min(new_destination_point.x, map.cols-1), 0);
  new_destination_point.y = std::max( std::min(new_destination_point.y, map.rows-1), 0);
  
  std::vector<cv::Point> predictions = expandDestination(
    map, map_resolution,
    object_point, new_destination_point,
    debug_map
  );

  bool is_obstacle = false;

  for (size_t prediction_idx = 0; prediction_idx < predictions.size(); prediction_idx++)
  {
    is_obstacle = checkObstacleBetween(object_point, predictions[prediction_idx], map, &debug_map);
    if (is_obstacle)
    {
      break;
    }
  }

  return !is_obstacle;

}

int LinearMotionModel::updatePrediction(  cv::Mat &map, float map_resolution,
                                          cv::Point object_point, cv::Point destination_point, float distance, 
                                          cv::Point &object_point_out, cv::Point &destination_point_out,
                                          cv::Mat &debug_map
                                        )
{
  current_object_point_ = object_point;
  current_destination_point_ = destination_point;

  if (is_first_)
  {
    // TODO: check if it's a good idea
    previous_destination_point_ = current_destination_point_;
    is_first_ = false;
  }

  float remaining_distance = distance;
  float new_distance = distance;
  size_t loop_count = 0;

  int status = 0;
  int avoid_infinit_loop_counter = 0;
  while (remaining_distance > 0 && loop_count < WAYPOINT_LOOP_LIMIT)
  {
    loop_count++;
    ROS_INFO("Prediction iter: %d", loop_count);

    // std::cout << "destination_point: " << destination_point << std::endl;
    if  (
          updateWayPoint(   
            map, map_resolution,
            object_point, destination_point, remaining_distance,
            object_point_out,  destination_point_out, new_distance,
            debug_map
          )
        )
    {
      ROS_ERROR("waypoing update error");
      status = 1;
      break;
    }
    // ROS_INFO("update prediction distance:%f", new_distance);

    if (object_point == destination_point)
    {
      ROS_WARN("Object point and destination point same");
      break;
    }

    remaining_distance = new_distance;
    object_point = object_point_out;
    destination_point = destination_point_out;

    if (remaining_distance <= 0 || loop_count < WAYPOINT_LOOP_LIMIT)
    {
      // if it's at the end
      cv::Point new_destination_point;
      bool is_destination_feasible = checkObjectDestinationFeasibility(
        object_point, 
        destination_point, 
        map,
        map_resolution,
        new_destination_point,
        debug_map
      );

      if (!is_destination_feasible)
      {
        ROS_WARN("goal close to obstacle");
        std::cout << object_point << "-" << destination_point << std::endl;
        std::cout << object_point << "-" << new_destination_point << std::endl;
        cv::circle(debug_map, new_destination_point, 15, 100);
        // std::cout << "destination_point: " << destination_point << std::endl;
        // std::cout << "new_destination_point: " << new_destination_point << std::endl;
      
        remaining_distance = cv::norm(new_destination_point - object_point) * map_resolution;
        destination_point = new_destination_point;
        // std::cout << "Infeasible destination, applying motion model again, distance increased by: " << remaining_distance << std::endl;
        
      }
    }

    // if (remaining_distance <= 0)
    // {
    //   if (avoid_infinit_loop_counter<20)
    //   {
    //     avoid_infinit_loop_counter++;
    //     cv::Point object_vector = destination_point - current_object_point_;
    //     float distance_from_object = cv::norm(object_vector) * map_resolution;
        
    //     if (distance_from_object < (float)PREDICTION_LOOKAHEAD_DISTANCE)
    //     {
    //       // add some length so that you are fixed distance from the original object
    //       remaining_distance = ((float)PREDICTION_LOOKAHEAD_DISTANCE-distance_from_object) * DESTINATION_EXTENTION_PERCENTAGE; 

    //       cv::Point object_vector_normalized(object_vector.x , object_vector.y);
    //       object_vector_normalized = object_vector_normalized / cv::norm(object_vector_normalized); 

    //       float remaining_distance_pixels = remaining_distance / map_resolution;
    //       destination_point.x += round(object_vector_normalized.x * remaining_distance_pixels);
    //       destination_point.y += round(object_vector_normalized.y * remaining_distance_pixels);

    //       object_vector = destination_point - current_object_point_;
    //       distance_from_object = cv::norm(object_vector) * map_resolution;

    //       if (distance_from_object > PREDICTION_LOOKAHEAD_DISTANCE)
    //       {
    //         ROS_WARN("Prediction (%f) distance greater than lookahead (%f)", distance_from_object, PREDICTION_LOOKAHEAD_DISTANCE);
    //       }
    //     }
    //   }
    //   else
    //   {
    //     // probably it is not a good destination
    //     return 1;
    //   }
    // }
  }

  previous_destination_point_ = destination_point_out;

  cv::circle(debug_map, object_point, 8, 255);
  cv::circle(debug_map, destination_point, 5, 255);
  
  return status;
}

std::vector<cv::Point> LinearMotionModel::expandDestination(
  cv::Mat &map, float map_resolution,
  cv::Point object_point, cv::Point destination_point,
  cv::Mat &debug_map
)
{
  cv::Point2f RP_unit_vector(
    destination_point.x - object_point.x,
    destination_point.y - object_point.y
  );
  RP_unit_vector = RP_unit_vector / cv::norm(RP_unit_vector);

  cv::Point2f normal_unit_vector(
    RP_unit_vector.y, -RP_unit_vector.x
  );

  cv::Point2f ray_endpoint1, ray_endpoint2;
  ray_endpoint1 = cv::Point2f(destination_point.x, destination_point.y) + OBSTACLE_RAYCASTING_PERTURBATION * normal_unit_vector;
  ray_endpoint2 = cv::Point2f(destination_point.x, destination_point.y) - OBSTACLE_RAYCASTING_PERTURBATION * normal_unit_vector;

  if (debug_map.total())
  { 
    cv::line(
      debug_map, 
      cv::Point(round(ray_endpoint1.x), round(ray_endpoint1.y)),
      cv::Point(round(ray_endpoint2.x), round(ray_endpoint2.y)), 
      cv::Scalar(255), 1, CV_AA
    );
  }

  // add perturbations to the destination points to get a number of rays
  std::vector<cv::Point> predictions;
  cv::LineIterator prediction_line_iterator(
    map, 
    cv::Point(round(ray_endpoint1.x), round(ray_endpoint1.y)),
    cv::Point(round(ray_endpoint2.x), round(ray_endpoint2.y))
  );

  cv::LineIterator it = prediction_line_iterator;
  for (size_t i = 0; i < prediction_line_iterator.count; i++, it++)
  {
    int x = it.pos().x;
    int y = it.pos().y;
    x = std::max( std::min(x, map.cols-1), 0);
    y = std::max( std::min(y, map.rows-1), 0);
    predictions.push_back(
      cv::Point(x, y)
    );
  }

  return predictions;
}

int LinearMotionModel::updateWayPoint(  cv::Mat &map, float map_resolution,
                                        cv::Point object_point, cv::Point destination_point, float distance,
                                        cv::Point &object_point_out, cv::Point &destination_point_out, float &remaining_distance_out,
                                        cv::Mat &debug_map
                                      )
{
  // default case: no obstacles, the point are all right
  object_point_out = object_point;
  destination_point_out = destination_point;
  remaining_distance_out = 0;

  // visualize the direction of the object
  cv::line(debug_map, object_point, destination_point, cv::Scalar(100), 1, CV_AA);

  cv::Point2f RP_unit_vector(
    destination_point.x - object_point.x,
    destination_point.y - object_point.y
  );
  RP_unit_vector = RP_unit_vector / cv::norm(RP_unit_vector);

  std::vector<cv::Point> predictions = expandDestination(
    map, map_resolution,
    object_point, destination_point,
    debug_map
  );

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
      // debug_map.at<uint8_t>(it.pos()) = 100;
    }
  }
  // for debug
  size_t num_obstacles_old = obstacles.size();
  
  obstacles = expandObstacleVector(map, map_resolution, obstacles);
  
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
        // debug_map.at<uint8_t>(obstacles[obstacle_idx]) = 0;
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
    cv::HoughLines(obstacle_image, lines, OBSTACLE_INFLATION * 1.2 / map_resolution, CV_PI/180.0*2.0, obstacle_count*0.45, 0, 0);

    // vector<Vec4i> lines;
    // HoughLinesP(obstacle_image, lines, 1, CV_PI/180, 15, 3, 1 );

    if (lines.size())
    {
      // for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
      // {
      //   float rho = lines[line_idx][0];
      //   float theta = lines[line_idx][1];
      //   ROS_INFO("number of lines: %d, line %d, theta:%f rho:%f ",lines.size(), line_idx, theta*180/M_PI, rho);
      // }

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
        cv::line(debug_map, pt1, pt2, cv::Scalar(50), 1, CV_AA);

        // use some clearance from actual obstacle point
        cv::Point obstacle_point(
          obstacles[0].x - RP_unit_vector.x * 3, 
          obstacles[0].y - RP_unit_vector.y * 3
        );

        // find the waypoint of person and prediction
        if ( chooseObstacleDirection(
                object_point, 
                destination_point,
                obstacle_point,
                map,
                map_resolution,
                theta,
                object_point_out,
                destination_point_out,
                distance, 
                remaining_distance_out
            )
          )
        {
          return 1;
        }

        // visualize the new waypoints
        cv::line(debug_map, object_point_out, destination_point_out, cv::Scalar(150), 1, CV_AA);
      }
    }
    else
    {
      // no line found even if there are obstacles
      ROS_WARN("Could not find obstacle line, consider changing hough threshold");

      // find the new waypoint of the destination, which is just clearance distance way from obstacle
      // angle of the vector RP (Robot to Prediction)
      float RP_theta = atan2(
        destination_point.y - object_point.y,
        destination_point.x - object_point.x
      );
      float clearance = OBSTACLE_CLEARANCE_DISTANCE / map_resolution;
      destination_point_out.x = round(obstacles[0].x - clearance * cos(RP_theta));
      destination_point_out.y = round(obstacles[0].y - clearance * sin(RP_theta));
      destination_point_out.x = std::max( std::min(destination_point_out.x, map.cols-1), 0);
      destination_point_out.y = std::max( std::min(destination_point_out.y, map.rows-1), 0);

      object_point_out = object_point;
      remaining_distance_out = 0;  
      return 1;  
    }
  }
  
  // visualize the robot and the person
  cv::circle(debug_map, object_point, 8, 255);
  cv::circle(debug_map, destination_point, 5, 255);

  return 0;

}

bool LinearMotionModel::backoff(
        cv::Point object_point, 
        cv::Point obstacle_point, 
        float backing_off_angle,
        cv::Mat &map,
        float map_resolution,
        cv::Point &object_point_out
)
{
  bool is_obstacle = false;
  for (size_t obstacle_check_idx = 0; obstacle_check_idx < 3; obstacle_check_idx++)
  {
    float clearance = OBSTACLE_CLEARANCE_DISTANCE / map_resolution;
    float clearance_2 = clearance * 2;
    object_point_out.x = round(obstacle_point.x + clearance_2 * cos(backing_off_angle));
    object_point_out.y = round(obstacle_point.y + clearance_2 * sin(backing_off_angle));
    object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
    object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);

    cv::LineIterator backing_off_line_iterator(
      map, 
      cv::Point(round(obstacle_point.x), round(obstacle_point.y)),
      cv::Point(round(object_point_out.x), round(object_point_out.y))
    );
    cv::LineIterator it = backing_off_line_iterator;

    cv::Point new_obstacle_point;
    for (size_t i = 0; i < backing_off_line_iterator.count; i++, it++)
    {
      if (map.at<uint8_t>(it.pos()) == 255)
      {
        is_obstacle = true;
        new_obstacle_point = it.pos();
        break;
      }
    }

    if (is_obstacle)
    {
      float two_obstacle_distance = cv::norm(obstacle_point - new_obstacle_point);
      if (two_obstacle_distance < (MINIMUM_DISTANCE_BETWEEN_OBSTACLES / map_resolution))
      {
        continue;
      }
      else
      {
        // back-off so that you are right between the obstacles
        float new_backoff_distance = two_obstacle_distance / 2.0;
        // don't back off completely, go only half way of the new obstacle
        // float new_backoff_distance = cv::norm(new_obstacle_point - obstacle_point) / 2.0;
        object_point_out.x = round(obstacle_point.x + new_backoff_distance * cos(backing_off_angle));
        object_point_out.y = round(obstacle_point.y + new_backoff_distance * sin(backing_off_angle));
        object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
        object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);
        is_obstacle = false;
        break;
      }
    }
    else
    {
      object_point_out.x = round(obstacle_point.x + clearance * cos(backing_off_angle));
      object_point_out.y = round(obstacle_point.y + clearance * sin(backing_off_angle));
      object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
      object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);
      break;
    }
  }

  return is_obstacle;
}

int LinearMotionModel::chooseObstacleDirection(
                                                  cv::Point object_point, 
                                                  cv::Point destination_point,
                                                  cv::Point obstacle_point,
                                                  cv::Mat &map,
                                                  float map_resolution,
                                                  float obstacle_orientation,
                                                  cv::Point &object_point_out,
                                                  cv::Point &destination_point_out,
                                                  float distance,
                                                  float &remaining_distance_out
                                                )
{
  int chosen_idx = -1;
  
  cv::Point2f object_vector(
    destination_point.x - object_point.x,
    destination_point.y - object_point.y
  );
  object_vector /= cv::norm(object_vector);

  float RP_theta = atan2(
    object_vector.y,
    object_vector.x
  );

  // two angles of the obstacle line
  float ol_theta[2];
  ol_theta[0] = atan2(
    -cos(obstacle_orientation),
    sin(obstacle_orientation)
  );
  ol_theta[1] = oppositeAngle(ol_theta[0]);

  // vector normal to the obstacle (needed for backing off)
  cv::Point2f obstacle_normal_vectors[2];
  obstacle_normal_vectors[0] = cv::Point2f(-sin(ol_theta[0]), cos(ol_theta[0]));
  obstacle_normal_vectors[1] = cv::Point2f(sin(ol_theta[0]), -cos(ol_theta[0]));

  float object2obstacle_normal_angle[2] = {
    acos(obstacle_normal_vectors[0].dot(object_vector)),
    acos(obstacle_normal_vectors[1].dot(object_vector))
  };

  float obstacle_normal_angles[2] = {
    atan2(obstacle_normal_vectors[0].y, obstacle_normal_vectors[0].x),
    atan2(obstacle_normal_vectors[1].y, obstacle_normal_vectors[1].x)
  };

  // choose max cuz you don't want to go through the obstacle
  size_t backing_off_angle_idx = (object2obstacle_normal_angle[0] > object2obstacle_normal_angle[1]) ? 0 : 1; 
  // float backing_off_angle = (object2obstacle_normal_angle[0] > object2obstacle_normal_angle[1]) ?
  //                           obstacle_normal_angles[0] : obstacle_normal_angles[1];

  // but the backing off directions based on priority
  float backing_off_angles[2] = {
    obstacle_normal_angles[backing_off_angle_idx],
    obstacle_normal_angles[1 - backing_off_angle_idx]
  };

  bool is_obstacle = false;

  for (size_t angle_idx = 0; angle_idx < 2; angle_idx++)
  {
    float backing_off_angle = backing_off_angles[angle_idx];
    
    is_obstacle = backoff(
      object_point, 
      obstacle_point, 
      backing_off_angle,
      map,
      map_resolution,
      object_point_out
    );

    if (!is_obstacle)
    {
      break;
    }
    else
    {
      ROS_WARN("backing off direction infeasible, taking another");
    }
  }

  if (is_obstacle)
  {
    ROS_ERROR("both backing off infeasible");
    return 1;
  }


  cv::Mat R_w_rp = theta2RotationMatrix(RP_theta);
  cv::Mat R_w_vl1 = theta2RotationMatrix(ol_theta[0]);
  cv::Mat R_w_vl2 = theta2RotationMatrix(ol_theta[1]);

  cv::Mat R_rp_vl1 = R_w_rp.t() * R_w_vl1;
  cv::Mat R_rp_vl2 = R_w_rp.t() * R_w_vl2;

  // angles between the vectors RP(object direction) and OL (Obstacle line)
  float angle_object_wall[2];
  angle_object_wall[0] = rotationMatrix2Theta(R_rp_vl1);
  angle_object_wall[1] = rotationMatrix2Theta(R_rp_vl2);

  // move remaining distance away from the robot waypoint towards the obstacle direction (wall following)
  float covered_distance_pixels = sqrt(
    pow(object_point_out.x - object_point.x, 2) + pow(object_point_out.y - object_point.y, 2)
  );

  if (covered_distance_pixels == 0)
  {
    ROS_ERROR("Backing off error");
    object_point_out = object_point;
    remaining_distance_out = 0;
    return 1;
  }

  float covered_distance_meters = covered_distance_pixels * map_resolution;
  // output
  remaining_distance_out = distance - covered_distance_meters;
  
  if (remaining_distance_out <= 0)
  {
    ROS_ERROR("Remaining distance error: covered more than required!!!");
    // revert the update
    object_point_out = object_point;
    remaining_distance_out = 0;
    return 1;
  }

  float remaining_distance_pixels = remaining_distance_out / map_resolution;
  
  // ROS_INFO(
  //   "covered_distance_pixels: %f, covered_distance_meters: %f, remaining_distance_out: %f, remaining_distance_pixels",
  //   covered_distance_pixels, covered_distance_meters, remaining_distance_out, remaining_distance_pixels
  // );

  cv::Point destination_points[2];

  for (size_t i = 0; i < 2; i++)
  {
    destination_points[i].x = round(object_point_out.x + remaining_distance_pixels * cos(ol_theta[i]));
    destination_points[i].y = round(object_point_out.y + remaining_distance_pixels * sin(ol_theta[i])); 
    destination_points[i].x = std::max( std::min(destination_points[i].x, map.cols-1), 0);
    destination_points[i].y = std::max( std::min(destination_points[i].y, map.rows-1), 0);
  }

  cv::Point2f original_object_destination_vector(
    current_destination_point_.x - current_object_point_.x,
    current_destination_point_.y - current_object_point_.y
  );
  original_object_destination_vector /= cv::norm(original_object_destination_vector);

  cv::Point2f current_object_destinations_vector[2];

  size_t num_going_back=0;
  std::cout << "Original vector: " << original_object_destination_vector << std::endl;
  for (size_t i = 0; i < 2; i++)
  {
    current_object_destinations_vector[i] = cv::Point2f(
      destination_points[i].x - object_point_out.x,
      destination_points[i].y - object_point_out.y
    );
    current_object_destinations_vector[i] /= cv::norm(current_object_destinations_vector[i]);

    float object_vector_angle = acos(original_object_destination_vector.dot(current_object_destinations_vector[i])) * 180 / M_PI;
    std::cout << "New vector " << i << ": " << current_object_destinations_vector[i] << std::endl;
    std::cout << "Current to new angle: " << object_vector_angle << std::endl;

    if  (
          fabs(object_vector_angle) > 130
        )
    {
      chosen_idx = 1 - i;
      num_going_back++;
    }
  }

  if (num_going_back >= 2)
  {
    ROS_ERROR("both the lines are going back, which is impossible!!!");
    return 1;
  }

  if (!num_going_back)
  {
    // none of the directions are going back, choose among them using the cost function
    std::vector<size_t> viable_obstacle_indices;
    getViablDirections(
      object_point_out,
      ol_theta,
      map,
      map_resolution,
      viable_obstacle_indices
    );

    if (viable_obstacle_indices.size() == 0)
    {
      ROS_ERROR("BOTH OBSTACLE_DIRECTIONS STUCK");
      return 1;
    }
    else if (viable_obstacle_indices.size() == 1)
    {
      ROS_WARN("one obstacle direction led to deadend");
      chosen_idx = viable_obstacle_indices[0];
    }
    else
    {
      float object2destinations_distance[2];
      float previous2currentdestinations_distance[2];
      float angles_to_original_vector[2];
      for (size_t i = 0; i < 2; i++)
      {
        object2destinations_distance[i] = sqrt(
          pow(destination_points[i].x - current_object_point_.x, 2) +
          pow(destination_points[i].y - current_object_point_.y, 2)
        ) * map_resolution;
      
        previous2currentdestinations_distance[i] = sqrt(
          pow(destination_points[i].x - previous_destination_point_.x, 2) +
          pow(destination_points[i].y - previous_destination_point_.y, 2)
        ) * map_resolution;

        // the direction of vector of the candidate object to destination should be along the same direction as the original object to destination
        angles_to_original_vector[i] = vectorAngle(
          cv::Point2f(
            current_destination_point_.x - current_object_point_.x,
            current_destination_point_.y - current_object_point_.y
          ),

          cv::Point2f(
            destination_points[i].x - object_point_out.x,
            destination_points[i].y - object_point_out.y
          )
        );

      }
  
      // all three costs for the two directions
      #define NUM_COSTS 4
      float costs[2][NUM_COSTS];
      for (size_t i = 0; i < 2; i++)
      {
        costs[i][0] = fabs(angle_object_wall[i]) / M_PI;
        costs[i][1] = previous2currentdestinations_distance[i] / PREDICTION_LOOKAHEAD_DISTANCE;
        costs[i][2] = 1.0/(1.0 + (object2destinations_distance[i] / PREDICTION_LOOKAHEAD_DISTANCE)) * 2.0;
        costs[i][3] = angles_to_original_vector[i] / M_PI;
      }
  
      // winners for the three costs
      int cost_winners[NUM_COSTS];
      for (size_t j = 0; j < NUM_COSTS; j++)
      {
        if ( fabs(costs[0][j] - costs[1][j]) < NORMALIZED_COST_THRESHOLD)
        {
          // there is no clear winner
          cost_winners[j] = 0;
        }
        else
        {
          cost_winners[j] = costs[1][j] < costs[0][j] 
                            ? 1 
                            : -1;  
        }
      }
  
      float total_cost[2] = {0, 0};
      float cost_weights[NUM_COSTS] = {
        10, 
        2, // 2, 
        0, // 1,
        3
      };
  
      for (size_t i = 0; i < 2; i++)
      {
        for (size_t j = 0; j < NUM_COSTS; j++)
        {
          if (j == 0)
          {
            if (cost_winners[0])
            {
              // we have a clear winner (the cost wasn't similar)
              total_cost[i] += cost_weights[j] * costs[i][j];
            }
          }
          else if (
                    cost_winners[j] && cost_weights[j]
                  )
          {
            // we have a clear winner (the cost wasn't similar)
            ROS_ERROR("Using cost %d to break tie", j);
            total_cost[i] += cost_weights[j] * costs[i][j];
          }
        }
      }
  
      ROS_INFO(
        "Wall1: %f, Opposite wall: %f",
        ol_theta[0] * 180 / M_PI,
        ol_theta[1] * 180 / M_PI
      );
      ROS_INFO("Winners: %d, %d, %d, %d", cost_winners[0], cost_winners[1], cost_winners[2], cost_winners[3]);
      ROS_INFO(
        "Cost[0]: %f, %f, %f, %f, Cost[1]: %f, %f, %f, %f", 
        costs[0][0], costs[0][1], costs[0][2], costs[0][3],
        costs[1][0], costs[1][1], costs[1][2], costs[1][3]
      );
  
      // TODO: filter out any prediction that is going towards the object
      chosen_idx = (total_cost[1] < total_cost[0]) ? 1 : 0;
    }
  }
  destination_point_out = destination_points[chosen_idx];

  ROS_INFO(
    "Robot: %f, Chosen wall: %f, Opposite wall: %f",
    RP_theta * 180 / M_PI,
    ol_theta[chosen_idx] * 180 / M_PI,
    ol_theta[1-chosen_idx] * 180 / M_PI
  );

  return 0;
}

float LinearMotionModel::oppositeAngle(float angle)
{
  // find the direction of the unit vector in the opposite direction
  return atan2(-sin(angle), -cos(angle));
}

std::vector<cv::Point> LinearMotionModel::expandObstacleVector(cv::Mat &map, float map_resolution, std::vector<cv::Point> obstacle_points)
{
  std::vector<cv::Point> expanded_obstacle_points;
  std::queue<cv::Point> bfs_queue;
  // the hash is pixel index row_idx * cols + col_idx
  std::map<uint32_t, bool> explored_pixels;

  for (size_t i = 0; i < obstacle_points.size(); i++)
  {
    uint32_t obstacle_hash = obstacle_points[i].y * map.cols + obstacle_points[i].x;
    if ( explored_pixels.find(obstacle_hash) == explored_pixels.end() )
    {
      // add to queue only if it's not already there (duh!!!)
      bfs_queue.push(obstacle_points[i]);
      explored_pixels.insert(std::make_pair(obstacle_hash, true));
    }
  }

  while (!bfs_queue.empty())
  {
    int bfs_length = bfs_queue.size();
    cv::Point obstacle_point = bfs_queue.front();
    bfs_queue.pop();
    if (bfs_length == bfs_queue.size())
      ROS_ERROR("queue pop didn't work");
    expanded_obstacle_points.push_back(obstacle_point);

    std::vector<cv::Point> neighbours;
    neighbours.reserve(8);

    // index offset for eight connected neighbours
    int index_offset[8][2] = {
      {-1, -1}, {-1, 0}, {-1, 1},
      {0, -1}, {0, 1},
      {+1, -1}, {+1, 0}, {+1, 1}
    };

    for (size_t neighbour_idx = 0; neighbour_idx < 8; neighbour_idx++)
    {
      int offset_x = index_offset[neighbour_idx][0];
      int offset_y = index_offset[neighbour_idx][1];

      cv::Point neighbour_point(obstacle_point.x + offset_x, obstacle_point.y + offset_y);

      uint32_t obstacle_hash = neighbour_point.y * map.cols + neighbour_point.x;
      // distance from the first obstacle point
      cv::Point obstacle_vector = neighbour_point - obstacle_points[0];
      cv::Mat obstacle_vector_mat(2, 1, CV_32F);
      obstacle_vector_mat.at<float>(0, 0) = obstacle_vector.x;
      obstacle_vector_mat.at<float>(1, 0) = obstacle_vector.y;
      float distance = cv::norm(obstacle_vector_mat, cv::NORM_L1) * map_resolution;
      if  ( 
            explored_pixels.find(obstacle_hash) == explored_pixels.end() &&
            neighbour_point.x > 0 &&
            neighbour_point.x < map.cols &&
            neighbour_point.y > 0 &&
            neighbour_point.y < map.rows &&
            map.at<uint8_t>(neighbour_point) == 255 &&
            distance < OBSTACLE_INFLATION * 6
          )
      {
        bfs_queue.push(neighbour_point);
        explored_pixels.insert(std::make_pair(obstacle_hash, true));
      }
    }
  }

  return expanded_obstacle_points;
}