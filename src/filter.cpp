#include "filter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


Filter::Filter(int size)
{
  points = new cv::Point3f[size];
  iterator = 0;
  length = size;
}

int Filter::addPoint(cv::Point3f point)
{
  points[iterator%length] = point;
  iterator++;
  return 0;
}

cv::Point3f Filter::getFilter()
{
  cv::Point3f avg = cv::Point3f(0,0,0);
  for (int i=0 ; i<length ; i++)
  {
    avg.x += points[i].x;
    avg.y += points[i].y;
    avg.z += points[i].z;
  }
  avg.x = avg.x / (double)length;
  avg.y = avg.y / (double)length;
  avg.z = avg.z / (double)length;
  return avg;
}