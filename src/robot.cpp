#include "filter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "robot.hpp"
#include "config.h"


cv::Point2f Robot::getRobotPose()
{
	return robot_poses->getFilter();
}

cv::Point2f Robot::getHumanPose()
{
	return human_poses->getFilter();
}

Robot::Robot(){
	robot_poses = new Filter(ROBOT_FILTER_SIZE);
	human_poses = new Filter(PERSON_FILTER_SIZE);
	destination_pose = new Filter(DESTINATION_FILTER_SIZE);
	human_prev_pose = cv::Point2f(0,0);
	human_prev_vel = cv::Point2f(0,0);
}

//-------------------------avg_dest is return of this function ----------------------
int Robot::calculateDistination(cv::Point2f human_pose, cv::Point2f& avg_dest)
{
	//-----------------------adding new point to human pose history--------------------
	human_poses->addPoint(human_pose);

  //-----------------------gitting pos history average-------------------------------
  cv::Point2f human_avg_pose = human_poses->getFilter(); 

  double speedX = human_avg_pose.x - human_prev_pose.x; 
  double speedY = human_avg_pose.y - human_prev_pose.y;        
   if (speedX == 0 && speedY == 0){
    speedX = human_prev_vel.x;
    speedY = human_prev_vel.y;
  }
  human_prev_pose = cv::Point2f(human_avg_pose.x,human_avg_pose.y);
  human_prev_vel = cv::Point2f(speedX,speedY);
  //----------------avoid deviding by 0----------------------------------------------
  if (speedX*speedX + speedY*speedY == 0){   
    return 1;
  }
  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
  destination_pose->addPoint(cv::Point2f(human_avg_pose.x+speedX*2 + speedXU*followdist
  													, human_avg_pose.y+speedY*2 + speedYU*followdist
  													));
  avg_dest = destination_pose->getFilter();
  return 0;
}

int Robot::followingBehhavior()
{
	return 0;
}

int Robot::publishCmdVel()
{
	return 0;
}




int getOtherRobotPoseBlob (robot_t *robot, Stg::Pose &otherRobot, ModelBlobfinder *blob, bool isFront)
{
    cv::Mat robot_pose = pose2TransformationMatrix(robot->prevPoseMine);

    double centerPointX = (blob->GetBlobs()[0].left + blob->GetBlobs()[0].right) / 2.0;
    double centerPointY = (blob->GetBlobs()[0].top + blob->GetBlobs()[0].bottom) / 2.0;
    // double x = range / f * (centerPointX - maxblobx/2);
    // double y = range; // sqrt(range*range - x*x);
    std::cout << "FOV: " << blob->fov * 180 / M_PI << std::endl;
    double degree = getBlobBearing( centerPointX);

    // double range = blob->GetBlobs()[0].range;
    double blobStartAngle = getBlobBearing( blob->GetBlobs()[0].left);
    double blobEndAngle = getBlobBearing( blob->GetBlobs()[0].right);
    // the start of laser is at the back of robot
    double laserStartAngle = isFront ? M_PI + blobStartAngle : blobStartAngle ;
    double laserEndAngle = isFront ? M_PI + blobEndAngle: blobEndAngle;
    
    laserStartAngle = atan2(sin(laserStartAngle), cos(laserStartAngle));
    laserEndAngle = atan2(sin(laserEndAngle), cos(laserEndAngle));
    laserStartAngle = laserStartAngle>=0 ? laserStartAngle : laserStartAngle + 2*M_PI;
    laserEndAngle = laserEndAngle>=0 ? laserEndAngle : laserEndAngle + 2*M_PI;

    // debug
    // std::cout << " Blob Start: " << blobStartAngle * 180 / M_PI 
    //           << " Blob End: " << blobEndAngle * 180 / M_PI 
    //           << std::endl
    //           << " Laser Start: " << laserStartAngle * 180 / M_PI 
    //           << " Laser End: " << laserEndAngle * 180 / M_PI 
    //           << std::endl;

    double range = getRangeFromLaser(robot, laserStartAngle, laserEndAngle);
    if (range == 0)
    {
      return 1;
    }

    // std::cout << "Estimated range: " << range
    //           << " Actual Range: " << blob->GetBlobs()[0].range
    //           << std::endl;

    
    // ------------------------ Exact relative ------------------------ //
    cv::Mat r1_position_global = pose2HomgeneousVector( robot->otherRobot->GetGlobalPose() );
    cv::Mat r1_position_local = robot_pose.inv() * r1_position_global;

    double exact_degree = M_PI - (
                                atan2 (
                                  robot->robotPose.y - robot->otherRobot->GetGlobalPose().y,
                                  robot->robotPose.x - robot->otherRobot->GetGlobalPose().x
                                  
                                ) + robot->pos->GetPose().a);
                          

    // ------------------------ Estimated relative ------------------------ //
    // use exact range
    double exact_range = sqrt(
      pow(robot->robotPose.y - robot->otherRobot->GetGlobalPose().y, 2) +
      pow(robot->robotPose.x - robot->otherRobot->GetGlobalPose().x, 2)
    );
    double relative_x_est = -range * cos(degree);
    double relative_y_est = range * sin(degree);
    
    if (isFront)
    {    
      relative_x_est *=-1;
      relative_y_est *= -1;
    }

    cv::Mat relative_est(3, 1, CV_32F);
    relative_est.at<float>(0, 0) = relative_x_est;
    relative_est.at<float>(1, 0) = relative_y_est;
    relative_est.at<float>(2, 0) = 1;

    std::cout << "Exact: " << r1_position_local.t() << ", Estimated: " << relative_est.t() << std::endl;
    // std::cout << "Exact range: " << exact_range << ", Estimated: " << range << std::endl;

    cv::Mat global_est = robot_pose * relative_est;
    
    otherRobot = Stg::Pose(global_est.at<float>(0, 0), global_est.at<float>(1, 0),0,0);
    
    // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
    // transformation from r0 to world frame
    cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
    Stg::Pose poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(otherRobot));
    ModelPosition::Waypoint wp(poseOther_w, Color("green"));

    ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    return 0;
}