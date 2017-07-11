#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "stage.hh"

cv::Mat pose2TransformationMatrix(Stg::Pose origin);
cv::Mat pose2HomgeneousVector(Stg::Pose pose);
Stg::Pose homogeneousVector2Pose(cv::Mat homogeneousVector);
cv::Mat xytheta2TransformationMatrix(cv::Mat xytheta);
cv::Mat xytheta2TransformationMatrix(cv::Point3f xytheta);