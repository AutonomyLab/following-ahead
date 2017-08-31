#ifndef LINEAR_MOTION_MODEL_HPP
#define LINEAR_MOTION_MODEL_HPP

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

class LinearMotionModel
{
private:
    cv::Point current_object_point_;
    cv::Point current_destination_point_;
    cv::Point previous_destination_point_;
    bool is_first_;
public:
    int updateWayPoint  (   cv::Mat &map, float map_resolution,
                            cv::Point object_point, cv::Point destination_point, float distance, 
                            cv::Point &object_point_out, cv::Point &destination_point_out, float &remaining_distance_out,
                            cv::Mat &debug_map
                        );

    /**
     * @param object_point_out, destination_point_out: updated waypoints
     */
    int chooseObstacleDirection(
                                    cv::Point object_point, 
                                    cv::Point destination_point,
                                    cv::Point obstacle_point,
                                    cv::Mat &map,
                                    float map_resolution,
                                    float obstacle_orientation,
                                    cv::Point &object_point_out,
                                    cv::Point &destination_point_out,
                                    float distance,
                                    float &remaining_distance_out
                                );

    int updatePrediction(   cv::Mat &map, float map_resolution,
                            cv::Point object_point, cv::Point destination_point, float distance, 
                            cv::Point &object_point_out, cv::Point &destination_point_out,
                            cv::Mat &debug_map
                        );
    /**
     *
     * @brief backing off behavior, accounting for obstacles
     */
    bool backoff(
        cv::Point object_point, 
        cv::Point obstacle_point, 
        float backing_off_angle,
        cv::Mat &map,
        float map_resolution,
        cv::Point &object_point_out
    );

    /**
     *
     * @brief checks if the object (person) to her destination is good (close to or beyond obstacles)
     */
    static bool checkObjectDestinationFeasibility(
        cv::Point object_point, 
        cv::Point destination_point, 
        cv::Mat &map,
        float map_resolution,
        cv::Point &new_destination_point,
        cv::Mat &debug_map
    );

    /**
     *
     * @brief expand predictions for raycasting between object and destination
     */
    static std::vector<cv::Point> expandDestination(
      cv::Mat &map, float map_resolution,
      cv::Point object_point, cv::Point destination_point,
      cv::Mat &debug_map
    );

    /**
     *
     * @brief check obstacle between two points
     */
    static bool checkObstacleBetween(cv::Point point1, cv::Point point2, cv::Mat &map, cv::Mat *debug_map=NULL);

    /**
     * @brief return the angle of the vector that is in the other direction (-> to <-)
     */
    float oppositeAngle(float angle);

    /**
     * @brief expand the number of obstacle pixels using BFS
     */
    std::vector<cv::Point> expandObstacleVector(cv::Mat &map, float map_resolution, std::vector<cv::Point> obstacle_points);
    LinearMotionModel(); 
};


#endif  