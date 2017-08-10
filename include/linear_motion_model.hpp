#ifndef LINEAR_MOTION_MODEL_HPP
#define LINEAR_MOTION_MODEL_HPP

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

class LinearMotionModel
{
private:
public:
    int updateWayPoint  (   cv::Mat &map, float map_resolution,
                            cv::Point object_point, cv::Point destination_point, float distance, 
                            cv::Point &object_point_out, cv::Point &destination_point_out, float &remaining_distance_out,
                            cv::Mat &debug_map
                        );

    /**
     * @return orientation of the obstacle vector that we have to follow
     */
    float chooseObstacleDirection(float RP_theta, float ol_theta);

    /**
     * @brief return the angle of the vector that is in the other direction (-> to <-)
     */
    float oppositeAngle(float angle);

    LinearMotionModel(){}; 
};


#endif  