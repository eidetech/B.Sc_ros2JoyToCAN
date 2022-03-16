#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "quintic.h"

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

    // TODO add desc
    void calcCartesianPosVelAcc();

    // Add all the points in the trajectory to the pt vectors
    void plan();

    // Function for adding points to the pt vectors
    void addPoints(float x, float z);

    // Prints the current list of points
    void printPoints();

    // Vectors that holds [x,z] points for the path planning
    std::vector<float> pt_x;
    std::vector<float> pt_z;

    int pt_len = 9;

    MatrixXf posVelAccTime;

    

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

// Variables for calculating the cartesian position, velocity and acceleration
float t0_ = 0;
float t1_ = 0;
float t_sum_ = 0;
float x0_ = 0;
float x1_ = 0;
float z0_ = 0;
float z1_ = 0;
float v0_x_ = 0;
float v1_x_ = 0;
float v0_z_ = 0;
float v1_z_ = 0;
float a0_x_ = 0;
float a1_x_ = 0;
float a0_z_ = 0;
float a1_z_ = 0;
};

#endif