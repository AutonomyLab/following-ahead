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

  debug_map = map.clone();
  
  float remaining_distance = distance;
  float new_distance = distance;
  size_t loop_count = 0;

  int status = 0;
  int avoid_infinit_loop_counter = 0;
  while (remaining_distance > 0)
  {
    ROS_INFO("Prediction iter: %d", loop_count++);
    
    if (
          updateWayPoint(   
            map, map_resolution,
            object_point, destination_point, remaining_distance,
            object_point_out,  destination_point_out, new_distance,
            debug_map
          )
      )
    {
      status = 1;
      break;
    }
    ROS_INFO("update prediction distance:%f", new_distance);
    
    if (object_point == destination_point)
    {
      ROS_WARN("Object point and destination point same");
      break;
    }

    remaining_distance = new_distance;
    object_point = object_point_out;
    destination_point = destination_point_out;

    if (remaining_distance <= 0)
    {
      if (avoid_infinit_loop_counter<20)
      {
        avoid_infinit_loop_counter++;
        cv::Point object_vector = destination_point - current_object_point_;
        float distance_from_object = cv::norm(object_vector) * map_resolution;
        
        if (distance_from_object < (float)PREDICTION_LOOKAHEAD_DISTANCE)
        {
          // add some length so that you are fixed distance from the original object
          remaining_distance = ((float)PREDICTION_LOOKAHEAD_DISTANCE-distance_from_object) * DESTINATION_EXTENTION_PERCENTAGE; 

          cv::Point object_vector_normalized(object_vector.x , object_vector.y);
          object_vector_normalized = object_vector_normalized / cv::norm(object_vector_normalized); 

          float remaining_distance_pixels = remaining_distance / map_resolution;
          destination_point.x += round(object_vector_normalized.x * remaining_distance_pixels);
          destination_point.y += round(object_vector_normalized.y * remaining_distance_pixels);

          object_vector = destination_point - current_object_point_;
          distance_from_object = cv::norm(object_vector) * map_resolution;

          if (distance_from_object > PREDICTION_LOOKAHEAD_DISTANCE)
          {
            ROS_WARN("Prediction (%f) distance greater than lookahead (%f)", distance_from_object, PREDICTION_LOOKAHEAD_DISTANCE);
          }
        }
      }
      else
      {
        // probably it is not a good destination
        return 1;
      }
    }
  }
  previous_destination_point_ = destination_point_out;

  cv::circle(debug_map, object_point, 8, 255);
  cv::circle(debug_map, destination_point, 5, 255);
  
  return status;
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

  cv::Point2f normal_unit_vector(
    RP_unit_vector.y, -RP_unit_vector.x
  );

  cv::Point2f ray_endpoint1, ray_endpoint2;
  ray_endpoint1 = cv::Point2f(destination_point.x, destination_point.y) + OBSTACLE_RAYCASTING_PERTURBATION * normal_unit_vector;
  ray_endpoint2 = cv::Point2f(destination_point.x, destination_point.y) - OBSTACLE_RAYCASTING_PERTURBATION * normal_unit_vector;

  cv::line(
    debug_map, 
    cv::Point(round(ray_endpoint1.x), round(ray_endpoint1.y)),
    cv::Point(round(ray_endpoint2.x), round(ray_endpoint2.y)), 
    cv::Scalar(150), 1, CV_AA
  );

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
      // debug_map.at<uint8_t>(it.pos()) = 127;
    }
  }
  // for debug
  size_t num_obstacles_old = obstacles.size();
  
  obstacles = expandObstacleVector(map, map_resolution, obstacles);
  
  // for debug
  if (obstacles.size() < num_obstacles_old )
  {
    ROS_ERROR("obs size: %d", obstacles.size());
  }
  if (obstacles.size() > num_obstacles_old + 10)
  {
    ROS_INFO("Obstacle increased by %d", obstacles.size() - num_obstacles_old);
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
    cv::HoughLines(obstacle_image, lines, OBSTACLE_INFLATION * 1.1 / map_resolution, CV_PI/180.0*2.0, obstacle_count*0.45, 0, 0);

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
  // cv::circle(debug_map, object_point, 8, 255);
  // cv::circle(debug_map, destination_point, 5, 255);

  return 0;

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
  float backing_off_angle = (object2obstacle_normal_angle[0] > object2obstacle_normal_angle[1]) ?
                            obstacle_normal_angles[0] : obstacle_normal_angles[1];

  float clearance = OBSTACLE_CLEARANCE_DISTANCE / map_resolution;
  object_point_out.x = round(obstacle_point.x + clearance * cos(backing_off_angle));
  object_point_out.y = round(obstacle_point.y + clearance * sin(backing_off_angle));
  object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
  object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);

  cv::LineIterator backing_off_line_iterator(
    map, 
    cv::Point(round(obstacle_point.x), round(obstacle_point.y)),
    cv::Point(round(object_point_out.x), round(object_point_out.y))
  );
  cv::LineIterator it = backing_off_line_iterator;

  bool is_obstacle = false;
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
    // don't back off completely, go only half way of the new obstacle
    float new_backoff_distance = cv::norm(new_obstacle_point - obstacle_point) / 2.0;
    object_point_out.x = round(obstacle_point.x + new_backoff_distance * cos(backing_off_angle));
    object_point_out.y = round(obstacle_point.y + new_backoff_distance * sin(backing_off_angle));
    object_point_out.x = std::max( std::min(object_point_out.x, map.cols-1), 0);
    object_point_out.y = std::max( std::min(object_point_out.y, map.rows-1), 0);
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
  
  ROS_INFO(
    "covered_distance_pixels: %f, covered_distance_meters: %f, remaining_distance_out: %f, remaining_distance_pixels",
    covered_distance_pixels, covered_distance_meters, remaining_distance_out, remaining_distance_pixels
  );

  cv::Point destination_points[2];

  for (size_t i = 0; i < 2; i++)
  {
    destination_points[i].x = round(object_point_out.x + remaining_distance_pixels * cos(ol_theta[i]));
    destination_points[i].y = round(object_point_out.y + remaining_distance_pixels * sin(ol_theta[i])); 
    destination_points[i].x = std::max( std::min(destination_points[i].x, map.cols-1), 0);
    destination_points[i].y = std::max( std::min(destination_points[i].y, map.rows-1), 0);
  }

  float object2destinations_distance[2];
  for (size_t i = 0; i < 2; i++)
  {
    object2destinations_distance[i] = sqrt(
      pow(destination_points[i].x - current_object_point_.x, 2) +
      pow(destination_points[i].y - current_object_point_.y, 2)
    ) * map_resolution;
  }

  float previous2currentdestinations_distance[2];
  for (size_t i = 0; i < 2; i++)
  {
    previous2currentdestinations_distance[i] = sqrt(
      pow(destination_points[i].x - previous_destination_point_.x, 2) +
      pow(destination_points[i].y - previous_destination_point_.y, 2)
    ) * map_resolution;
  }

  // all three costs for the two directions
  float costs[2][3];
  for (size_t i = 0; i < 2; i++)
  {
    costs[i][0] = fabs(angle_object_wall[i]) / M_PI;
    costs[i][1] = previous2currentdestinations_distance[i] / PREDICTION_LOOKAHEAD_DISTANCE;
    costs[i][2] = 1.0/(1.0 + (object2destinations_distance[i] / PREDICTION_LOOKAHEAD_DISTANCE)) * 2.0;
  }
  
  // winners for the three costs
  int cost_winners[3];
  for (size_t j = 0; j < 3; j++)
  if ( fabs(costs[0][j] - costs[1][j]) < NORMALIZED_COST_THRESHOLD)
  {
    cost_winners[j] = 0;
  }
  else
  {
    cost_winners[j] = costs[1][j] < costs[0][j] ? 1 : -1;  
  }
  
  object2destinations_distance[1] > object2destinations_distance[0] ? 1 : 0;

  float total_cost[2] = {0, 0};
  float cost_weights[3] = {10, 2, 1};
  for (size_t i = 0; i < 2; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      if (cost_winners[j])
      {
        // we have a clear winner (the cost wasn't similar)
        total_cost[i] += cost_weights[j] * costs[i][j];
      }
    }
  }

  ROS_INFO(
    "Wall1: %f, Opposite wall: %f",
    ol_theta[0] * 180 / M_PI,
    ol_theta[1] * 180 / M_PI
  );
  ROS_INFO("Winners: %d, %d, %d", cost_winners[0], cost_winners[1], cost_winners[2]);
  ROS_INFO(
    "Cost[0]: %f, %f, %f, Cost[1]: %f, %f, %f", 
    costs[0][0], costs[0][1], costs[0][2],
    costs[1][0], costs[1][1], costs[1][2]
  );

  // TODO: filter out any prediction that is going towards the object
  int chosen_idx = (total_cost[1] < total_cost[0]) ? 1 : 0;
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