#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <stage_ros/Blobs.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"

class SimulationInterface
{
public:
	SimulationInterface(ros::NodeHandle nh): nh_(nh)
	{ 	
		pub_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("/person_follower/groundtruth_pose", 1);

		sub_blob_front_ = nh_.subscribe<stage_ros::Blobs>(
			"/robot_0/blob_0", 0, 
			boost::bind(&SimulationInterface::blobCallback, this, 0, _1)
		);
		sub_blob_back_ = nh_.subscribe<stage_ros::Blobs>(
			"/robot_0/blob_1", 1, 
			boost::bind(&SimulationInterface::blobCallback, this, 1, _1)
		);

		sub_robot_pose_ = nh_.subscribe("/robot_0/base_pose_ground_truth", 1, &SimulationInterface::robotPoseCallback, this);
		sub_person_pose_ = nh_.subscribe("/robot_1/base_pose_ground_truth", 1, &SimulationInterface::personPoseCallback, this);
	}

	double getBlobBearing(double blobCoord, double imageWidth)
	{
		return (blobCoord / (imageWidth-1))*g_fov - g_fov/2.0;
	}

	double getYawFromQuaternion(geometry_msgs::Quaternion quaternion)
	{
		tf::Quaternion q(
			quaternion.x,
			quaternion.y,
			quaternion.z,
			quaternion.w
		);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		return yaw;
	}

	void robotPoseCallback(const nav_msgs::Odometry::ConstPtr &robot_pose_msg)
	{
		robot_pose_.x = robot_pose_msg->pose.pose.position.x;
		robot_pose_.y = robot_pose_msg->pose.pose.position.y;
		robot_pose_.z = getYawFromQuaternion(robot_pose_msg->pose.pose.orientation);
	}

	void personPoseCallback(const nav_msgs::Odometry::ConstPtr &person_pose_msg)
	{
		person_pose_.x = person_pose_msg->pose.pose.position.x;
		person_pose_.y = person_pose_msg->pose.pose.position.y;
		person_pose_.z = getYawFromQuaternion(person_pose_msg->pose.pose.orientation);
	}

	void blobCallback(int isFront, const boost::shared_ptr<stage_ros::Blobs const> &blobMsg)
	{

		// ROS_INFO("Callback from %d, count: %d", isFront, blobMsg->detections.size());
		// no blob
		// if (counter%10 !=0)
		//   return 0;
		double maxblobx = blobMsg->width;
		// std::cout << "maxhight: " << blobMsg->width << std::endl;

	
		if (!blobMsg->detections.size())
			return;

	
	    double centerPointX = (blobMsg->detections[0].left + blobMsg->detections[0].right) / 2.0;
		// double centerPointY = (blobMsg->detections[0].top + blobMsg->detections[0].bottom) / 2.0;

		double degree = getBlobBearing(centerPointX, maxblobx);
		ROS_INFO("BEARING: %f", degree * 180.0 / M_PI);
		double blobStartAngle = getBlobBearing( blobMsg->detections[0].left, maxblobx);
    	double blobEndAngle = getBlobBearing( blobMsg->detections[0].right, maxblobx);
		// the start of laser is at the back of robot
		double laserStartAngle = isFront ? M_PI + blobStartAngle : blobStartAngle ;
		double laserEndAngle = isFront ? M_PI + blobEndAngle: blobEndAngle;
    
		laserStartAngle = atan2(sin(laserStartAngle), cos(laserStartAngle));
		laserEndAngle = atan2(sin(laserEndAngle), cos(laserEndAngle));
		laserStartAngle = laserStartAngle>=0 ? laserStartAngle : laserStartAngle + 2*M_PI;
		laserEndAngle = laserEndAngle>=0 ? laserEndAngle : laserEndAngle + 2*M_PI;

	
		double range = blobMsg->detections[0].range;
		if (range == 0)
		{
		  return;
		}


		// ------------------------ Estimated relative ------------------------ //

		double relative_x_est = -range * cos(degree);
		double relative_y_est = range * sin(degree);

		if (isFront)
		{    
		  relative_x_est *=-1;
		  relative_y_est *= -1;
		}

		ROS_INFO("Relative estimate: %f, %f", relative_x_est, relative_y_est);

		cv::Mat relative_est(3, 1, CV_32F);
		relative_est.at<float>(0, 0) = relative_x_est;
		relative_est.at<float>(1, 0) = relative_y_est;
		relative_est.at<float>(2, 0) = 1;
		
		geometry_msgs::TransformStamped msg;
		msg.header.stamp = blobMsg->header.stamp;

		msg.transform.translation.x = relative_x_est;
		msg.transform.translation.y = relative_y_est;
		// the z is range
		msg.transform.translation.z = range;
		// the quaternion x is the bearing angle
		msg.transform.rotation.x = degree;
		msg.transform.rotation.y = 0;
		msg.transform.rotation.z = 0;
		msg.transform.rotation.w = 1;

		pub_pose_.publish(msg);

		// cv::Mat robot_pose = xytheta2TransformationMatrix(robot_pose_);
		// cv::Mat global_person = robot_pose * relative_est;
		// otherRobot = Stg::Pose(global_est.at<float>(0, 0), global_est.at<float>(1, 0),0,0);

		// // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
		// // transformation from r0 to world frame
		// cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
		// Stg::Pose poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(otherRobot));
		// ModelPosition::Waypoint wp(poseOther_w, Color("green"));

		// ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
	}

protected:
	ros::NodeHandle nh_;
	ros::Publisher pub_pose_;
	ros::Subscriber sub_blob_front_;
	ros::Subscriber sub_blob_back_;
	ros::Subscriber sub_robot_pose_;
	ros::Subscriber sub_person_pose_;
	cv::Point3f robot_pose_;
	cv::Point3f person_pose_;
};


int main(int argc, char** argv )
{
	ros::init(argc, argv, "person_follower_node"); 
	ros::NodeHandle n;

	SimulationInterface simulation_interface(n);
		
	ros::spin();

	return 0; // ok
}
