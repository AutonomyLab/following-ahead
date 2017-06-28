#include "ros/ros.h"
#include "yolo2/ImageDetections.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
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

#include <sstream>

#include <limits> include <vector> include <iostream> include <cassert>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "pid.h"
#include "stage.hh"

using namespace Stg;

bool firstTime = true;
static const bool UseOtherRobotPose = false;
static const bool UseLaser = false;
static const double followdist = 4;
static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const double stopdist = 0.3;
static const int avoidduration = 10;
static const int numberOfPosHist = 30;
static const int numUpdateReferenceFrame = 30;
static const int imageWidth = 640;
static const int imageHeight = 480;
static const float fov = 60.0*M_PI/180.0;
static const float focalLengthX = 538.914261;
static const float focalLengthY = 504.416883;
static const float cameraPrincipalX = 311.027555;
static const float cameraPrincipalY = 260.575111;


int maxblobx = 80;
static bool isahead = true;


typedef struct {
  Stg::Pose robotPose;
  ModelPosition *pos;
  ModelRanger *laser;
  ModelBlobfinder *blobF;
  ModelBlobfinder *blobB;
  Stg::Pose prevPoseMine;

  // for visualization purpose, we save the absolute pose when the origin was fixed
  Stg::Pose robotPose_w;
  
  Stg::Pose posesAvgLastOther;
  Stg::Pose* posesOther;
  int TheposesOtherNum;
  Stg::Pose* lastDestination;
  int state; // 1 means blob detected, 0 not detected
  PID pidcruse;
  PID pidturn;
  Model* otherRobot;
  float degreeD;
  double distance;
  int avoidcount, randcount;

  int referenceLastUpdated;
} robot_t;


int counter = 0;
int MyBlobUpdate (robot_t*robot, bool isFront);
int BlobUpdateBack (Model *, robot_t *robot);
int BlobUpdateFront (Model *, robot_t *robot);
int LaserUpdate(Model *mod, robot_t *robot);
int PoseUpdate(Model *, robot_t *robot);
int setSpeed(robot_t *robot, Stg::Pose destination);
int getOtherRobotPoseBlob (robot_t *robot, Stg::Pose &otherRobot, ModelBlobfinder *blob, bool isFront);
int getOtherRobotPoseLaser(const Stg::ModelRanger::Sensor& sensor,  robot_t *robot, Stg::Pose &otherRobot);
int getDestinationBasedOnOtherObjectPoses(robot_t *robot, Stg::Pose otherRobot, Stg::Pose& avgDestinations);
int createWaypoint(robot_t *robot, Stg::Pose, char* color);

ros::Publisher pubPointCloud;
sensor_msgs::PointCloud::Ptr pointCloudMsg (new sensor_msgs::PointCloud);

double getBlobBearing( double blobCoord)
{
  // return ( blobCoord * fov / (imageWidth-1) - fov/2.0);
  return (blobCoord / (imageWidth-1))*fov;

}

float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
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
        vectPersonPoints.push_back(cv::Point2f(col, row));
        depthValues.push_back(cv_ptr->image.at<float>(row, col));
      }
    }

    // float medianDepth = median(depthValues);

    // for (
    //       int row=detectionMsg->detections[i].roi.y_offset; 
    //       row<detectionMsg->detections[i].roi.y_offset+detectionMsg->detections[i].roi.height;
    //       row++
    //     )
    // {
    //   for (
    //         int col=detectionMsg->detections[i].roi.x_offset; 
    //         col<detectionMsg->detections[i].roi.x_offset+detectionMsg->detections[i].roi.width;
    //         col++
    //       )
    //   {
    //     if (
    //           cv_ptr->image.at<float>(row, col) >= medianDepth-0.12 &&
    //           cv_ptr->image.at<float>(row, col) <= medianDepth+0.12
    //       )   
    //     {
    //       matPersonSegmented.at<uint8_t>(row, col) = 255;
    //     }
        
    //   }
    // }



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
  distortionCoeffs[0] = 0.176206;
  distortionCoeffs[1] = -0.363409;
  distortionCoeffs[2] = 0.001018;
  distortionCoeffs[3] =  -0.003290;
  distortionCoeffs[4] = 0.000000;

  cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix, distortionCoeffs);

  std::cout << "Depth image size: " <<  cv_ptr->image.size() << std::endl;
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

    // std::cout << getBlobBearing(pt.x * cameraMatrix.at<float>(0, 0)) * 180.0 / M_PI << std::endl;
  }

  undistortedPoints.clear();
  cv::undistortPoints(vectPersonPoints, undistortedPoints, cameraMatrix, distortionCoeffs);
  pointCloudMsg->header.frame_id = "world";
  // pointCloudMsg->height = pointCloudMsg->width = 1;
  pointCloudMsg->header.stamp = ros::Time::now();
  pointCloudMsg->points.clear();

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

    pointCloudMsg->points.push_back(point);
  }


  pubPointCloud.publish(pointCloudMsg);
  
  
  std::cout << std::endl;
}


// Stage calls robotthis when the model starts up
int main(int argc, char** argv )
{
  Robot myRobot;



  ros::init(argc, argv, "person_follower_node");
  ros::NodeHandle n;
  
  // ros::Publisher yoloEnablePub = n.advertise("vision/yolo2/enable", 1000);
  // yoloEnablePub.publish(std_msgs::Bool(true));

  message_filters::Subscriber<yolo2::ImageDetections> detectionSub(n, "vision/yolo2/detections", 1);
  message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "camera/depth_registered/sw_registered/image_rect", 1);
  typedef message_filters::sync_policies::ApproximateTime<yolo2::ImageDetections, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), detectionSub, depthSub);
  sync.registerCallback(boost::bind(&detectionCallback, _1, _2));
  // ros::Subscriber detection_sub = n.subscribe("/vision/yolo2/detections", 10, detectionCallback);

  pubPointCloud =  n.advertise<sensor_msgs::PointCloud>("person_cloud", 1);
  
  ros::spin();


  robot_t *robot = new robot_t();


  if (isahead){
    robot->pidturn.set(avoidturn, -avoidturn, avoidturn, 0.0100, 0.01);
    robot->pidcruse.set(cruisespeed*3, -cruisespeed, -cruisespeed  , .0100, 0.001);
  }

  robot->avoidcount = 0;
  robot->posesAvgLastOther  = Stg::Pose(0.,0.,0.,0.);
  robot->posesOther  = new Stg::Pose[numberOfPosHist];
  robot->TheposesOtherNum = 0;
  robot->lastDestination  = new Stg::Pose[numberOfPosHist];
  robot->randcount = 0;
  robot->distance = 0;
  robot->state = 0;
  robot->degreeD = 0;
  robot->referenceLastUpdated = 0;
  
  robot->robotPose = Stg::Pose(0, 0, 0, 0);
  robot->prevPoseMine = Stg::Pose(0, 0, 0, 0);
  robot->robotPose_w = robot->pos->GetPose();
  return 0; // ok
}


int myBlobUpdate ( robot_t *robot, bool isFront)
{
  ModelBlobfinder *blob;

  if (isFront)
    blob = robot->blobF;
  else
    blob = robot->blobB;
  double speedX = 0;
  double speedY = 0;
  // no blob
  // if (counter%10 !=0)
  //   return 0;
  maxblobx = blob->scan_width;
  std::cout << "maxhight: " << blob->scan_width << std::endl;

  if (UseOtherRobotPose){

    double x = robot->otherRobot->GetGlobalPose().x;
    double y = robot->otherRobot->GetGlobalPose().y;
    speedX = x - robot->posesOther[0].x; 
    speedY = y - robot->posesOther[0].y;          
    
    robot->posesOther[0] = Stg::Pose(x,y,speedX,speedY);

    double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
    double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
    robot->lastDestination[0] = Stg::Pose(x+speedX*2 + speedXU*followdist, y+speedY*2 + speedYU*followdist, 0.,0.);
    ModelPosition::Waypoint wp(robot->lastDestination[0],
                                             Color("blue"));
    setSpeed(robot,robot->lastDestination[0]);

  }
  else if (!blob->GetBlobs().size()){
    return 0;

    //---------------- prediction to do:
    // if (firstTime)
    //   return 0;
    // Stg::Pose speed = Stg::Pose(robot->posesAvgLastOther.z, robot->posesAvgLastOther.a,0,0);
    // // speedX = robot->posesAvgLastOther.z;
    // // speedY = robot->posesAvgLastOther.a;
    // Stg::Pose otherRobot = robot->posesAvgLastOther;
    // calPosBasedSpeed(speed ,otherRobot);
    // Stg::Pose avgDestinations;
    // if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations))
    //   return 0;
    // ModelPosition::Waypoint wp(otherRobot, Color("red"));
    // ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    // std::cout << "a other robot pos: " << otherRobot.x << " "<< otherRobot.y << std::endl;

    
    // setSpeed(robot,avgDestinations);

  }
  else{


    Stg::Pose otherRobot;
    if (getOtherRobotPoseBlob (robot, otherRobot, blob, isFront))
    {
      return 0;
    }

    Stg::Pose avgDestinations;
    if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations))
    {
      return 0;
    }

    setSpeed(robot,avgDestinations);
  }
}

int BlobUpdateFront(Model * mod, robot_t *robot)
{
  if (UseLaser)
    return 0;

  myBlobUpdate (robot, true);

  return 0;
}

int BlobUpdateBack(Model * mod, robot_t *robot)
{
  if (UseLaser)
    return 0;
  myBlobUpdate (robot, false);

  return 0; // run again
}

cv::Mat pose2TransformationMatrix(Stg::Pose origin)
{
  cv::Mat pose = cv::Mat::eye(3, 3, CV_32F);
  pose.at<float>(0, 0) = cos(origin.a);
  pose.at<float>(0, 1) = -sin(origin.a);
  pose.at<float>(1, 0) = -pose.at<float>(0, 1);
  pose.at<float>(1, 1) = pose.at<float>(0, 0);
  pose.at<float>(0, 2) = origin.x;
  pose.at<float>(1, 2) = origin.y;

  return pose;
}

cv::Mat pose2HomgeneousVector(Stg::Pose pose)
{
  cv::Mat homogeneousVector(3, 1, CV_32F);
  homogeneousVector.at<float>(0, 0) = pose.x;
  homogeneousVector.at<float>(1, 0) = pose.y;
  homogeneousVector.at<float>(2, 0) = 1.0f;

  return homogeneousVector.clone();
}

Stg::Pose homogeneousVector2Pose(cv::Mat homogeneousVector)
{
  assert(homogeneousVector.rows==3 && homogeneousVector.cols==1);
  return Stg::Pose( homogeneousVector.at<float>(0, 0),  homogeneousVector.at<float>(1, 0), 0, 0 );
}

Stg::meters_t getRangeFromLaser(robot_t *robot, double startAngle, double endAngle)
{
  // get the data
  std::vector<Stg::meters_t> scan = robot->laser->GetSensors()[0].ranges;
  // make the orientation of laser scan counter-clockwise (+ve) to the robot
  std::reverse(scan.begin(), scan.end());

  double angleIncrement = robot->laser->GetSensors()[0].fov/(double)(robot->laser->GetSensors()[0].sample_count-1);
  size_t startIdx = round(startAngle/angleIncrement);
  size_t endIdx = round(endAngle/angleIncrement);

  // debug
  // std::cout << "Non saturated laser" << std::endl;
  // for (size_t i=0; i<scan.size(); i++)
  // {
  //   if (scan[i]<8)
  //   {
  //     std::cout << i << " ";
  //   }
  // }
  // std::cout << std::endl;

  // std::cout << "Start: " << startIdx << " End: " << endIdx << std::endl;
  
  std::vector<Stg::meters_t> validScan;
  for (size_t i=startIdx; i!=endIdx+1; i++)
  {
    i = i % robot->laser->GetSensors()[0].sample_count;
    if (scan[i] < robot->laser->GetSensors()[0].range.max)
    {
      // std::cout << scan[i] << std::endl;
      validScan.push_back(scan[i]);
    }
  }
  
  if (validScan.size() == 0)
  {
    std::cout << "No valid scan" << std::endl;
    return 0;
  }

  // median
  size_t medianIdx = validScan.size()/2;
  std::nth_element(validScan.begin(), validScan.begin()+medianIdx, validScan.end());
  return validScan[medianIdx];
  // min
  // return *(std::min_element(validScan.begin(), validScan.end()));

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

 int getOtherRobotPoseLaser(const Stg::ModelRanger::Sensor& sensor,  robot_t *robot, Stg::Pose &otherRobot)
{
    // get the data
    const std::vector<Stg::meters_t>& scan = sensor.ranges;
  
    uint32_t sample_count = scan.size();
    if( sample_count < 1 )
        return 1;

    double laser_orientation =  robot->pos->GetPose().a - sensor.fov/2.0;
    double angle_increment = sensor.fov/(double)(sensor.sample_count-1);
    double other_x_avg = 0.0;
    double other_y_avg = 0.0;
    double num_point = 0.0;
    for (uint32_t i = 0; i < sample_count; i++) {
        // normalize the angle
        laser_orientation = atan2(sin(laser_orientation), cos(laser_orientation));
        
        double laser_x, laser_y;
        int laser_grid_x, laser_grid_y;

        laser_x =  robot->robotPose.x +  scan[i] * cos(laser_orientation);
        laser_y =  robot->robotPose.y +  scan[i] * sin(laser_orientation);
        
        // if (convertToGridCoords(laser_x, laser_y, laser_grid_x, laser_grid_y)) {
        //     continue;
        // }


        if ( scan[i] < (sensor.range.max - std::numeric_limits<float>::min()) ) {
          num_point++;
          other_x_avg += laser_x;
          other_y_avg += laser_y;
        }

        laser_orientation += angle_increment;
    }
    std::cout << "number point: " << num_point << std::endl;
    if (num_point){
      std::cout << "get other robot pos xy: "<< other_x_avg/num_point << " " << other_y_avg/num_point << std::endl;
      otherRobot = Stg::Pose(other_x_avg/num_point,other_y_avg/num_point,robot->otherRobot->GetGlobalPose().z,0.);
      ModelPosition::Waypoint wp(Stg::Pose(other_x_avg/num_point,other_y_avg/num_point,robot->otherRobot->GetGlobalPose().z,0.), Color("green"));
      ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    }
    else {
     return 1;
    }
    

    return 0;
}

Stg::Pose filterWithPriority(Stg::Pose* poseVector , int posNum)
{
  Stg::Pose avgDestinations= Stg::Pose(0,0,0,0);
  double devide = 1;
  for (int i=0 ; i<numberOfPosHist-1 ; i++)
  {
    devide *= 0.5;
    avgDestinations.x += poseVector[(posNum-i+ numberOfPosHist)%numberOfPosHist].x * devide;
    avgDestinations.y += poseVector[(posNum-i + numberOfPosHist)%numberOfPosHist].y * devide;
  }
  avgDestinations.x += poseVector[(posNum+numberOfPosHist-(numberOfPosHist-1))%numberOfPosHist].x * devide;
  avgDestinations.y += poseVector[(posNum + numberOfPosHist -(numberOfPosHist-1))%numberOfPosHist].y * devide;

  return avgDestinations;
}

Stg::Pose filterWithoutPriority(Stg::Pose* poseVector)
{
  Stg::Pose avgDestinations= Stg::Pose(0,0,0,0);
  for (int i=0 ; i<numberOfPosHist ; i++)
  {
    avgDestinations.x += poseVector[i].x;
    avgDestinations.y += poseVector[i].y;
  }
  avgDestinations.x = avgDestinations.x / (double)numberOfPosHist;
  avgDestinations.y = avgDestinations.y / (double)numberOfPosHist;
  return avgDestinations;
}


// inspect the ranger data and decide what to do
int laserCounter =0;
int getDestinationBasedOnOtherObjectPoses(robot_t *robot, Stg::Pose otherRobot, Stg::Pose& avgDestinations)
{
  int posNum = robot->TheposesOtherNum % numberOfPosHist;
  double xTemp = otherRobot.x;
  double yTemp =  otherRobot.y;
  if (firstTime){
    robot->posesAvgLastOther = Stg::Pose(xTemp,yTemp,0,0);
    for (int i=0; i<numberOfPosHist ; i++){
       robot->posesOther[i] = Stg::Pose(xTemp,yTemp,0,0);
       robot->lastDestination[i] = Stg::Pose(xTemp,yTemp,0,0);
       std::cout << "XY temp: " << xTemp << " " << yTemp << " "<< std::endl;
    }
    firstTime =false;
  }
  robot->posesOther[posNum] = Stg::Pose(xTemp,yTemp,0,0);



  //-----------------------gitting pos history average-------------------------------
  
  Stg::Pose historyAVG = filterWithoutPriority(robot->posesOther); 

  // double sumX = 0.0;
  // double sumY = 0.0;
  // double devider = 1;
  // for (int i=0; i<numberOfPosHist - 1 ; i++){
  //   devider *= 0.5;
  //   sumX += robot->posesOther[i].x * devider;
  //   sumY += robot->posesOther[i].y * devider; 
  // }
  // sumX += robot->posesOther[numberOfPosHist-1].x * devider;
  // sumY += robot->posesOther[numberOfPosHist-1].y * devider; 

  // double x = sumX;
  // double y = sumY;


  double speedX = historyAVG.x - robot->posesAvgLastOther.x; 
  double speedY = historyAVG.y - robot->posesAvgLastOther.y;        
   if (speedX == 0 && speedY == 0){
    speedX = robot->posesAvgLastOther.z;
    speedY = robot->posesAvgLastOther.a;
  }
  robot->posesAvgLastOther = Stg::Pose(historyAVG.x,historyAVG.y,speedX,speedY);
  robot->posesOther[posNum] = Stg::Pose(xTemp,yTemp,0,0);
  if (speedX*speedX + speedY*speedY == 0){   //to avoid devide by 0
    return 1;
  }
  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
  robot->lastDestination[posNum] = Stg::Pose(historyAVG.x+speedX*2 + speedXU*followdist, historyAVG.y+speedY*2 + speedYU*followdist, 0.,0.);
  //-----------------------gitting dest history average focus on last destination---------

  avgDestinations = filterWithoutPriority(robot->lastDestination);

  // avgDestinations= Stg::Pose(0,0,0,0);
  // double devide = 1;
  // for (int i=0 ; i<numberOfPosHist-1 ; i++){
  //   devide *= 0.5;
  //   avgDestinations.x += robot->lastDestination[(posNum-i+ numberOfPosHist)%numberOfPosHist].x * devide;
  //   avgDestinations.y += robot->lastDestination[(posNum-i + numberOfPosHist)%numberOfPosHist].y * devide;

  // } 
  // avgDestinations.x += robot->lastDestination[(posNum+numberOfPosHist-(numberOfPosHist-1))%numberOfPosHist].x * devide;
  // avgDestinations.y += robot->lastDestination[(posNum + numberOfPosHist -(numberOfPosHist-1))%numberOfPosHist].y * devide;
  robot->TheposesOtherNum ++;
  return 0;
}

int FiterReferenceFrameUpdate(robot_t *robot)
{
  // transformation from frame 2 (current robot pose) to frame 1 (previous robot pose)
  cv::Mat T12 = pose2TransformationMatrix(robot->robotPose);
  // transformation from frame 2 to frame 1
  cv::Mat T21 = T12.inv();
  cv::Mat homogeneousVectorF1;
  cv::Mat homogeneousVectorF2;

  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->pos->GetPose());
  for (int i=0 ; i< numberOfPosHist ; i++){

    homogeneousVectorF1 = pose2HomgeneousVector(robot->lastDestination[i]);
    homogeneousVectorF2 = T21 * homogeneousVectorF1;
    robot->lastDestination[i] = homogeneousVector2Pose(homogeneousVectorF2);
    Stg::Pose poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->lastDestination[i]));
    createWaypoint(robot, poseOther_w, "cyan");

    homogeneousVectorF1 = pose2HomgeneousVector(robot->posesOther[i]);
    homogeneousVectorF2 = T21 * homogeneousVectorF1;
    robot->posesOther[i] = homogeneousVector2Pose(homogeneousVectorF2);
    std::cout << "filter reffrence points: " << robot->posesOther[i].x << " " << robot->posesOther[i].y;
    
    poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->posesOther[i]));
    createWaypoint(robot, poseOther_w, "red");

  }


}



int PoseUpdate(Model *, robot_t *robot)
{
  // robot->robotPose = robot->pos->GetPose();
  // return 0;
  // std::cout << "velocity(x,y): " <<((Stg::Pose)robot->pos->GetVelocity()).x << " " << ((Stg::Pose)robot->pos->GetGlobalVelocity()).y << std::endl;
  
  if (robot->referenceLastUpdated >= numUpdateReferenceFrame)
  {
    ((ModelPosition*)robot->otherRobot)-> waypoints.clear();

    FiterReferenceFrameUpdate(robot);

    robot->robotPose = Stg::Pose(0, 0, 0, 0);
    robot->prevPoseMine = Stg::Pose(0, 0, 0, 0);
    robot->referenceLastUpdated = 0;

    // just for visualization of waypoints
    robot->robotPose_w = robot->pos->GetPose();
  }
  else
  {
    double omega = ((Stg::Pose)robot->pos->GetVelocity()).a;
    double v = ((Stg::Pose)robot->pos->GetVelocity()).x;

    std::cout << "Y velocity: " << ((Stg::Pose)robot->pos->GetVelocity()).y << std::endl;

    robot->robotPose.x +=  v * cos(robot->prevPoseMine.a) / 10. ;
    robot->robotPose.y +=  v * sin(robot->prevPoseMine.a) / 10.;
    
    robot->prevPoseMine = robot->robotPose;
    robot->robotPose.a += omega / 10. ;
    robot->robotPose.a = atan2(sin(robot->robotPose.a), cos(robot->robotPose.a));
  }

  robot->referenceLastUpdated++;

  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
  Stg::Pose robotPose_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->robotPose));
  ModelPosition::Waypoint wp(robotPose_w, Color("yellow"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
  return 0;
}

int createWaypoint(robot_t *robot, Stg::Pose pose, char* color)
{
    ModelPosition::Waypoint wp(Stg::Pose(pose.x ,pose.y , 0.0, pose.a), Color(color));
   ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
}

int LaserUpdate(Model *, robot_t *robot)
{
  if (!UseLaser)
    return 0;


  Stg::Pose otherRobot;
  if (getOtherRobotPoseLaser (robot->laser->GetSensors()[0], robot, otherRobot)) // no position found
    return 0;

  Stg::Pose avgDestinations;
  if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations)) // will set avg destination
    return 0;
  setSpeed(robot, avgDestinations);

  return 0;
 }

int dummyCounter=0;

int setSpeed(robot_t *robot, Stg::Pose destination)
{
  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
  Stg::Pose destination_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(destination));
  ModelPosition::Waypoint wp(destination_w ,Color("blue"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
  
  if ((destination.x - robot->robotPose.x)==0) // Avoid devide by 0
    return 0;
  double m = (destination.y - robot->robotPose.y) / (destination.x - robot->robotPose.x);
  double degreeD = atan(m);

  if (destination.x < robot->robotPose.x && destination.y > robot->robotPose.y  )
    degreeD = M_PI + degreeD;  
  else if (destination.x < robot->robotPose.x && destination.y < robot->robotPose.y  )
    degreeD = -M_PI + degreeD;

  if (abs(degreeD -robot->degreeD) >= 180){
    std::cout << "degreed + .: " << degreeD -robot->degreeD <<std::endl;
    degreeD = robot->degreeD;
  }
  robot->degreeD = degreeD;
  std::cout << "m: "<< m << "atan: " <<  degreeD*180 / M_PI << std::endl;
  
  double Dturn = robot->robotPose.a - degreeD;

  if (Dturn >= M_PI)
    Dturn -= M_PI*2;
  else if (Dturn <= -M_PI)
     Dturn += M_PI*2;
  std::cout << "dturn: " << Dturn << std::endl;
  //set turn speed
  double Tspeed = robot->pidturn.calculate( 0 ,  Dturn , 0.01 );
  std::cout << "Tspeed: " << Tspeed << std::endl;
  robot->pos->SetTurnSpeed(Tspeed);

  // dummyCounter++;

  // if (dummyCounter%2)
  //   robot->pos->SetTurnSpeed(
  //     0.4
  //   );
  // else{
  //   robot->pos->SetTurnSpeed(
  //     -0.4
  //   );
  // }

  robot->distance = sqrt(pow(destination.x- robot->robotPose.x, 2) +  pow(destination.y - robot->robotPose.y, 2));
  std::cout<< "distance: " << robot->distance << std::endl;
  double Xspeed = robot->pidcruse.calculate(0 ,robot->distance, 0.01 );
  std::cout << "Xspeed: " << Xspeed << std::endl;
  robot->pos->SetXSpeed(Xspeed);
  // robot->pos->SetXSpeed(0.4);

  return 0; // run again
}


