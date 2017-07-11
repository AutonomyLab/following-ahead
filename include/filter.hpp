#ifndef FILTER_HPP
#define FILTER_HPP

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class Filter
{
private:
	 cv::Point3f *points;
	 unsigned int iterator;
	 size_t length;
public:
	Filter(){};
	Filter(int size);
	int addPoint(cv::Point3f);
	cv::Point3f getFilter();
};

#endif