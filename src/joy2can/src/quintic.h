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
    // Function for calculating trajectory with quintic polynomial (per axis)
    /*    Inputs:
            t0  ==> [s]     time at initial point
            t1  ==> [s]     time at final point
            t  ==>  [s]     current time
            p0  ==> [m]     position of initial point
            p1  ==> [m]     position of final point
            v0  ==> [m/s]   velocity at initial point
            v1  ==> [m/s]   velocity at final point
            a0  ==> [m/s^2] acceleration at initial point
            a1  ==> [m/s^2] acceleration at final point

        Ouputs (via getters):
            p   ==> [m]     array of position points
            v   ==> [m/s]   array of velocity points
            a   ==> [m/s^2] array of acceleration points
            t   ==> time array used to generate trajectory
    */
    void calcQuinticTraj(float t0, float t1, float t, float p0, float p1, float v0, float v1, float a0, float a1);
    
    // Getter functions
    float getPos();
    float getVel();
    float getAcc();

    // Vector and matrices for solving the six coefficients (x_0 -> x_5)
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