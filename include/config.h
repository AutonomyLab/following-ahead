#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>

#define DISTANCE_EPSILON 0.05
#define VELOCITY_ERROR_EPSILON 1.0
#define POSITION_ERROR_EPSILON 1.0
#define ORIENTATION_ERROR_EPSILON 1.0

#define PERSON_FILTER_SIZE 1
#define ROBOT_FILTER_SIZE 1
#define DESTINATION_FILTER_SIZE 30
#define LOOP_TIME 0.1
#define MAX_DEL_TIME 1.0
#define BLOB_UPDATE_TIME 1./120
#define NUM_UPDATE_REF_FRAME 300

#define CRUISE_SPEED 0.3
#define AVOID_SPEED 0.01
#define AVOID_TURN 0.9
#define MINI_FRONT_DISTANCE 1.0 // 0.6
#define STOP_DISTANCE 0.3
#define avoidduration 10
#define numberOfPosHist 1

#define NUM_STATES 6

// if the estimated velocity is lower than this, assume it to be 0
#define VELOCITY_THRESHOLD 0.01

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define NUM_PARTICLES 1000
// at the start, the particles should be spread out more, hence the factor
#define PARTICLE_INIT_NOISE_FACTOR_X 10
#define PARTICLE_INIT_NOISE_FACTOR_Y 20

#define PARTICLE_STOCHASTIC_VELOCITY_STDDEV 0.01
// TODO: The blob noise should be modelled in pixels, not meters
#define BLOB_NOISE_STDDEV_X 0.1
#define BLOB_NOISE_STDDEV_Y 0.1

#define DEPTH_LIMIT_TRACKING 1.0

// Kalman filter index:
#define X_T_IDX 0
#define Y_T_IDX 1
#define X_T_1_IDX 2
#define Y_T_1_IDX 3
#define VEL_IDX 4
#define THETA_IDX 5
// Infinite impulse response (IIR) filter for velocity and orientation
// higher these values, more importance to the new readings
#define VEL_IIR_ALPHA 0.3
#define THETA_IIR_ALPHA 0.85 // 0.7 

// process noise during the prediction step of EKF
#define X_T_PROCESS_NOISE_VAR 0.01
#define Y_T_PROCESS_NOISE_VAR 0.01
#define X_T_1_PROCESS_NOISE_VAR 0.01
#define Y_T_1_PROCESS_NOISE_VAR 0.01
#define VEL_PROCESS_NOISE_VAR 0.02
#define THETA_PROCESS_NOISE_VAR 0.2

// measurement noise during the correction step of EKF
// TODO: they are in meters, change them to be in pixels
#define X_T_MEASUREMENT_NOISE_VAR 0.1
#define Y_T_MEASUREMENT_NOISE_VAR 0.1

// initial error state covariance
#define X_T_INIT_ERROR_VAR 0.01
#define Y_T_INIT_ERROR_VAR 0.01
#define X_T_1_INIT_ERROR_VAR 0.01
#define Y_T_1_INIT_ERROR_VAR 0.01
#define VEL_INIT_ERROR_VAR 0.1
#define THETA_INIT_ERROR_VAR 1.7

// in rad^2
#define BEARING_ANGLE_ERROR_VAR 0.45	
#define BEARING_RANGE_ERROR_VAR 0.012


#define ROBOT_ORIENTATION_VARIANCE_SCALING 5.0
#define ROBOT_VELOCITY_VARIANCE_SCALING 0.5

// lookahead distance (m) for prediction
#define PREDICTION_LOOKAHEAD_DISTANCE 2.5 //4

#define MINIMUM_DISTANCE_BETWEEN_OBSTACLES 0.7

// maximum orientation change between two iterations (used for calculating probability of the measurement)
#define MAX_DEL_THETA 0.17453292519943295 // 10 degrees
#define MEASUREMENT_PROBABILITY_EPSILON 0.0001

#define TRANSPORT_PARTICLE_PROBABILITY_EPSILON 0.0001

#define STDDEV_EPSILON 0.0001

// probability of occupancy beyond which we consider it to be obstacle
#define OCCUPANCY_THRESHOLD 70
#define OBSTACLE_RAYCASTING_PERTURBATION 5
// distance to maintain from the obstacles (for the motion model)
#define OBSTACLE_CLEARANCE_DISTANCE 0.8 
// the normalized cost difference below this will count as a draw
#define NORMALIZED_COST_THRESHOLD 0.04
// dilation to inflate obstacles in map (in meters)
#define OBSTACLE_INFLATION 1.0 // 0.6
#define DESTINATION_EXTENTION_PERCENTAGE 1.2

// distance to obstacle that is considered feasible for destination
#define FEASIBLE_DESTINATION_TO_OBSTACLE_DISTANCE 0.7
#define WAYPOINT_LOOP_LIMIT 5
#define PERSON_LOST_TIMEOUT 2

#define PERSON_ORIENTATION_FILTER_SIZE 4

// distance to lookahead to see if there are deadends
#define DEADEND_LOOKAHEAD_DISTANCE 0.50

#define FOV 170.0

#define CAM_WRT_LASER_X -0.04
#define CAM_WRT_LASER_Y 0.0
#define CAM_WRT_LASER_Z 0.094
#define CAM_WRT_LASER_PITCH 5

static const float g_fov = FOV*M_PI/180.0;
static const float focalLengthX = 300.4547119140625;
static const float focalLengthY = 300.4547119140625;
static const float cameraPrincipalX = 157.3207550048828;
static const float cameraPrincipalY = 119.32881927490234;

#endif
