#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>

#define DISTANCE_EPSILON 0.01

#define PERSON_FILTER_SIZE 1
#define ROBOT_FILTER_SIZE 1
#define DESTINATION_FILTER_SIZE 30
#define LOOP_TIME 0.1
#define BLOB_UPDATE_TIME 1./120
#define NUM_UPDATE_REF_FRAME 300

#define FOLLOW_DIST 2.5
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
#define THETA_IIR_ALPHA 0.3

// process noise during the prediction step of EKF
#define X_T_PROCESS_NOISE_VAR 0.01
#define Y_T_PROCESS_NOISE_VAR 0.01
#define X_T_1_PROCESS_NOISE_VAR 0.01
#define Y_T_1_PROCESS_NOISE_VAR 0.01
#define VEL_PROCESS_NOISE_VAR 0.01
#define THETA_PROCESS_NOISE_VAR 0.01

// measurement noise during the correction step of EKF
// TODO: they are in meters, change them to be in pixels
#define X_T_MEASUREMENT_NOISE_VAR 0.1
#define Y_T_MEASUREMENT_NOISE_VAR 0.1

// initial error state covariance
#define X_T_INIT_ERROR_VAR 0.01
#define Y_T_INIT_ERROR_VAR 0.01
#define X_T_1_INIT_ERROR_VAR 0.01
#define Y_T_1_INIT_ERROR_VAR 0.01
#define VEL_INIT_ERROR_VAR 0.01
#define THETA_INIT_ERROR_VAR 0.01

// lookahead distance (m) for prediction
#define PREDICTION_LOOKAHEAD_DISTANCE 2.0

#define FOV 180.0

static const float g_fov = FOV*M_PI/180.0;
static const float focalLengthX = 538.914261;
static const float focalLengthY = 504.416883;
static const float cameraPrincipalX = 311.027555;
static const float cameraPrincipalY = 260.575111;

#endif
