#include <Eigen/Dense>

#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#pragma once

using namespace Eigen;

class TrajectoryPlanner
{
public:
    // Constructor
    TrajectoryPlanner();
    // Destructor
    ~TrajectoryPlanner();

    // Calculate the 
    void plan();

    // Matrix of [x,z] points to generate paths between
    ArrayXXf pt;
    int pt_rows = 2;

private:
// Trajectory generation parameters
float paint_vel_ = 1;       // [m/s] desired constant velocity when painting
float ramp_dist_ = 0.6;     // [m]   distance from stationary to start point
float ramp_time_ = 1.2;     // [s]   time from ramp_dist to start point
float turn_time_ = 1;       // [s]   time to turn around to pass wall one level above
float wall_width_ = 2.000;  // [m]   width of wall
float wall_height_ = 2.000; // [m]   heigth of wall
float wall_vStep_ = 0.500;  // [m]   vertical height step (vertical distance between horizontal lines
float x_offset_ = 0.200;    // [m]   offset from (0,0) in x direction
float z_offset_ = 0.200;    // [m]   offset from (0,0) in z direction

int floor_ = 0;             // [-]   keeps track of the current floor
float totalHeight_ = 0;     // [m]   keeps track of the total height
};

#endif