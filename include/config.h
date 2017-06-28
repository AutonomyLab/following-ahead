#ifndef CONFIG_H
#define CONFIG_H

#define PERSON_FILTER_SIZE 30
#define ROBOT_FILTER_SIZE 30
#define DESTINATION_FILTER_SIZE 30

static const double followdist = 4;
static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const double stopdist = 0.3;
static const int avoidduration = 10;
static const int numberOfPosHist = 30;
static const int numUpdateReferenceFrame = 30;
static const int imageWidth = 640;
static const int imageHeight = 480;
static const float fov = 60.0*M_PI/180.0;
static const float focalLengthX = 538.914261;
static const float focalLengthY = 504.416883;
static const float cameraPrincipalX = 311.027555;
static const float cameraPrincipalY = 260.575111;

#endif
