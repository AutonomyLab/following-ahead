#ifndef ORIENTATION_FILTER_HPP
#define ORIENTATION_FILTER_HPP

#include "filter.hpp"

/**
 *
 * @brief FIR filter for orientation, uses unit vector instead of angles to avoid wrap around issues
 */

class OrientationFilter: public Filter
{
public:
	OrientationFilter(int size);
	// @override
	int addPoint(float orientation);
	// @override
	float getFilter();

	void initialize(float orientation);

};
#endif