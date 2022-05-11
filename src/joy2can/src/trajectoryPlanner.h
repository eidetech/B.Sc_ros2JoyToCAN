#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "quintic.h"
#include "math.h"

#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#pragma once

#ifndef PI
#define PI 3.14159265359
#endif


using namespace Eigen;

class TrajectoryPlanner
{
public:
    // Constructor
    TrajectoryPlanner();
    // Destructor
    ~TrajectoryPlanner();

    // Calculates the system matrix containing all the QPP data for the specified trajectory
    void calcCartesianPosVelAcc();

    // Add all the points in the trajectory to the pt vectors
    void plan();

    // Function for adding points to the pt vectors
    void addPoints(float x, float z);

    // Prints the current list of points
    void printPoints();

    // Getters
    float get_x_offset();
    float get_z_offset();
    float get_wall_height();
    float get_wall_width();
    float get_ramp_dist();
    float get_outer_frame_width();
    float get_outer_frame_height();

    // Setters
    void set_x_offset(float x_offset);
    void set_z_offset(float z_offset);
    void set_wall_height(float wall_height);
    void set_wall_width(float wall_width);
    void set_ramp_dist(float ramp_dist);
    void set_outer_frame_width(float outer_frame_width);
    void set_outer_frame_height(float outer_frame_height);
    void set_vStep(float wall_vStep);

    // Reset calculation variables
    void reset();

    // Calculations
    void calc_vStep(float percent_overlap);

    // Vectors that holds [x,z] points for the path planning
    std::vector<float> pt_x;
    std::vector<float> pt_z;

    MatrixXf posVelAccTime;
    int N = 0;

private:
// Trajectory generation parameters
float paint_vel_ = 0.2;     // [m/s] desired constant velocity when painting
float ramp_dist_ = 0.3;     // [m]   distance from stationary to start point
float ramp_time_ = 2;       // [s]   time from ramp_dist to start point
float turn_time_ = 2;       // [s]   time to turn around to pass wall one level above
float wall_width_ = 1;      // [m]   width of wall
float wall_height_ = 1;     // [m]   heigth of wall
float wall_vStep_ = 0.1479; // [m]   vertical height step (vertical distance between horizontal lines

float outer_frame_width_ = 2;    // [m] outermost frame width defined by telescopic poles
float outer_frame_height_ = 1.5; // [m] outermost frame height defined by telescopic poles

float x_offset_ = 0.400;    // [m]   offset from (0,0) in x direction
float z_offset_ = 0.400;    // [m]   offset from (0,0) in z direction

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
float sprayStatus_ = 0;
};

#endif