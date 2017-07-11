#include "utils.hpp"

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

cv::Mat xytheta2TransformationMatrix(cv::Mat xytheta)
{
  return pose2TransformationMatrix(
    Stg::Pose(
      xytheta.at<float>(0, 0),
      xytheta.at<float>(1, 0),
      0,
      xytheta.at<float>(2, 0)
    )
  );
}

cv::Mat xytheta2TransformationMatrix(cv::Point3f xytheta)
{
  return pose2TransformationMatrix(
    Stg::Pose(
      xytheta.x,
      xytheta.y,
      0,
      xytheta.z
    )
  );
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