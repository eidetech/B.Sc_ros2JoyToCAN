#include <Eigen/Dense>
#include "math.h"

#ifndef QUINTIC_H
#define QUINTIC_H

#pragma once

using namespace Eigen;

class Quintic
{
    
public:
    // Constructor
    Quintic();

    // Destructor
    ~Quintic();
    
public:
    // Function for calculating trajectory with quintic polynomial
    void calcQuinticTraj(float t0, float t1, float t, float p0, float p1, float v0, float v1, float a0, float a1);
    
    // Getter functions
    float getPos();
    float getVel();
    float getAcc();

    // Matrices for solving the six coefficients (x_0 -> x_5)
    MatrixXf matA;
    VectorXf vecB;
    VectorXf x;
    
private:
    // Position, velocity and acceleration variables
    float p_ = 0;
    float v_ = 0;
    float a_ = 0;
};

#endif