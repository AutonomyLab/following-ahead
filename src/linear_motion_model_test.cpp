#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <algorithm>
#include "linear_motion_model.hpp"
#include "config.h"
#include "utils.hpp"

#define MAP_SIZE 1000

int main(int argc, char **argv)
{
	cv::Mat map = cv::Mat::zeros(MAP_SIZE, MAP_SIZE, CV_8UC1);
  LinearMotionModel person_motion_model;
	// obstacle
	cv::line(
		map, 
		cv::Point(
			MAP_SIZE/2 + 70,
			0
		), 
		cv::Point(
			MAP_SIZE/2 + 70,
			MAP_SIZE-1
		), 
		cv::Scalar(255), 
		2, CV_AA
	);

	cv::Point robot_image_coordinates(
		MAP_SIZE/2 + 50,
		MAP_SIZE/2 + 10
	);

	cv::Point prediction_image_coordinates(
		MAP_SIZE/2 + 100,
		MAP_SIZE/2 + 100
	);

	cv::Point new_robot_image_coordinates, new_prediction_image_coordinates;
	cv::Mat debug_map;
	float remaining_distance;

	person_motion_model.updateWayPoint(   
  	map, 0.1, 
    robot_image_coordinates, prediction_image_coordinates, 0.0, 
  	new_robot_image_coordinates, new_prediction_image_coordinates, remaining_distance,
  	debug_map
	);

	cv::Mat debug_map_flipped;
	cv::flip(debug_map, debug_map_flipped, 0);

	cv::imshow("window", debug_map_flipped);
	cv::waitKey();

	return 0;


}