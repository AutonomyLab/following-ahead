#include "utils.hpp"
#include <cmath>
#include <cfloat>


cv::Mat theta2RotationMatrix(float theta)
{
  cv::Mat rotation_matrix = cv::Mat(2, 2, CV_32F);
  rotation_matrix.at<float>(0, 0) = cos(theta);
  rotation_matrix.at<float>(0, 1) = -sin(theta);
  rotation_matrix.at<float>(1, 0) = -rotation_matrix.at<float>(0, 1);
  rotation_matrix.at<float>(1, 1) = rotation_matrix.at<float>(0, 0);

  return rotation_matrix;
}

float rotationMatrix2Theta(cv::Mat rotation_matrix)
{
  return atan2(
    rotation_matrix.at<float>(1, 0), 
    rotation_matrix.at<float>(1, 1)
  );
}

float vectorAngle(cv::Point2f vector1, cv::Point2f vector2)
{
  vector1 /= norm(vector1);
  vector2 /= norm(vector2);

  return acos(vector1.dot(vector2));
}

cv::Mat xytheta2TransformationMatrix(cv::Mat xytheta)
{
  float x = xytheta.at<float>(0, 0);
  float y = xytheta.at<float>(1, 0);
  float a = xytheta.at<float>(2, 0);

  cv::Mat pose = cv::Mat::eye(3, 3, CV_32F);
  pose.at<float>(0, 0) = cos(a);
  pose.at<float>(0, 1) = -sin(a);
  pose.at<float>(1, 0) = -pose.at<float>(0, 1);
  pose.at<float>(1, 1) = pose.at<float>(0, 0);
  pose.at<float>(0, 2) = x;
  pose.at<float>(1, 2) = y;

  return pose;
}

cv::Mat xytheta2TransformationMatrix(cv::Point3f xytheta)
{
  cv::Mat mat(3, 1, CV_32F);
  mat.at<float>(0, 0) = xytheta.x;
  mat.at<float>(1, 0) = xytheta.y;
  mat.at<float>(2, 0) = xytheta.z;

  return xytheta2TransformationMatrix(mat);
}

cv::Point3f transformPoint(tf::StampedTransform transform, cv::Point3f point)
{
  return transformPoint(static_cast<tf::Transform>(transform), point);

  // tf::Vector3 point_vect(point.x, point.y, 0);
  // tf::Vector3 transformed_point_vect = transform(point_vect);

  // if  (   std::isnan(transformed_point_vect.getX()) ||
  //         std::isnan(transformed_point_vect.getY()) ||
  //         std::isnan(transformed_point_vect.getZ())
  //     )
  // {
  //   ROS_ERROR("NaN in transformation");
  // }
  // return cv::Point3f(
  //   transformed_point_vect.getX(),
  //   transformed_point_vect.getY(),
  //   transformed_point_vect.getZ()
  // );
}

cv::Point3f transformPoint(tf::Transform transform, cv::Point3f point)
{
  tf::Vector3 point_vect(point.x, point.y, 0);
  tf::Vector3 transformed_point_vect = transform(point_vect);

  if  (   std::isnan(transformed_point_vect.getX()) ||
          std::isnan(transformed_point_vect.getY()) ||
          std::isnan(transformed_point_vect.getZ())
      )
  {
    ROS_ERROR("NaN in transformation");
  }

  return cv::Point3f(
    transformed_point_vect.getX(),
    transformed_point_vect.getY(),
    transformed_point_vect.getZ()
  );
}