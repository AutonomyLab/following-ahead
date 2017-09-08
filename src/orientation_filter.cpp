#include "orientation_filter.hpp"
#include <cmath>

OrientationFilter::OrientationFilter(int size): Filter(size)
{
	
}

int OrientationFilter::addPoint(float orientation)
{
	cv::Point3f point(
		cos(orientation), sin(orientation), 0
	);
	Filter::addPoint(point);	
}

float OrientationFilter::getFilter()
{
	cv::Point3f point = Filter::getFilter();
	return atan2(point.y, point.x);
}

void OrientationFilter::initialize(float orientation)
{
	cv::Point3f point(
		cos(orientation), sin(orientation), 0
	);
	for (size_t i = 0; i < length; i++)
	{
		Filter::addPoint(point);
	}
}