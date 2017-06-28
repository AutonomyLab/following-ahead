#ifndef FILTER_HPP
#define FILTER_HPP

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class Filter
{
private:
	 cv::Point2f *points;
	 unsigned int iterator;
	 size_t length;
public:
	Filter(){};
	Filter(int size);
	int addPoint(cv::Point2f);
	cv::Point2f getFilter();
};

#endif