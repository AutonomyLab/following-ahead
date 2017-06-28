#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "filter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class Robot
{
private:
	Filter* robot_poses;
	Filter* human_poses;
	Filter* destination_pose;
	cv::Point2f human_prev_pose;
	cv::Point2f human_prev_vel;
public:
	Robot();
	cv::Point2f getRobotPose();
	cv::Point2f getHumanPose();
	int relativePoseCallback();
	int kiniectPoseCallback();
	int calculateDistination(cv::Point2f, cv::Point2f&);
	int followingBehhavior();
	int publishCmdVel();
};


#endif	