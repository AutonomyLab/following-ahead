#ifndef CONFIG_H
#define CONFIG_H

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
#define NUM_STATES 11

// if the estimated velocity is lower than this, assume it to be 0
#define VELOCITY_THRESHOLD 0.01

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define FOV 180.0

static const float g_fov = FOV*M_PI/180.0;
static const float focalLengthX = 538.914261;
static const float focalLengthY = 504.416883;
static const float cameraPrincipalX = 311.027555;
static const float cameraPrincipalY = 260.575111;
static const float Q0 = 0.05;
static const float Q1 = 0.05;
static const float Q2 = 0.09;
static const float Q3 = 0.15;
static const float Q4 = 0.19;
static const float Q5 = 0.05; // 0.15;
static const float Q6 = 0.05; // 0.15;
static const float Q7 = 0.15; // 0.25;
static const float Q8 = 0.15; // 0.25;

static const float R0 = 0.05;
static const float R1 = 0.02;
static const float R2 = 0.05;
static const float R3 = 0.05;

static const float P0 = 0.01;
static const float P1 = 0.01;
static const float P2 = 0.01;
static const float P3 = 0.01;
static const float P4 = 0.01;
static const float P5 = 0.01; // 1; // Error position of person x
static const float P6 = 0.01; // 1; // Error position of person y
static const float P7 = 0.1; // 0.5;
static const float P8 = 0.1; // 0.5;


#endif
