#pragma once
#include "global.h" // global variables

// cluster compact constraint
double compactParam = 20; // \delta, whos unit is voxel. todo: move to config file.
double g_distance_step = 0.5; // meter.
// angle constraint
double g_angleDifference = PI/3 - 0.05;
// dense interpolation
bool g_fpInterpolation = false;

// trajectories
std::vector<std::vector<Point_2>> g_rbtTrajectories; // need initialize
std::vector<std::vector<iro::SE2>> g_camTrajectories; // need initialize
float g_camera_height = 1.1; // meter.

int g_plan_iteration = 0;

std::vector<cv::Point> g_scene_boundary;