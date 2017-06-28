#include "filter.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


Filter::Filter(int size)
{
  points = new cv::Point2f[size];
  iterator = 0;
  length = size;
}

int Filter::addPoint(cv::Point2f point)
{
  points[iterator%length] = point;
  return 0;
}

cv::Point2f Filter::getFilter()
{
  cv::Point2f avg = cv::Point2f(0,0);
  for (int i=0 ; i<length ; i++)
  {
    avg.x += points[i].x;
    avg.y += points[i].y;
  }
  avg.x = avg.x / (double)length;
  avg.y = avg.y / (double)length;
  return avg;
}