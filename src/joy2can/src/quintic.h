#include <Eigen/Dense>
#include "math.h"

#ifndef QUINTIC_H
#define QUINTIC_H

#pragma once

using namespace Eigen;

class Quintic
{
    
public:
    Quintic();
    ~Quintic();

    void calcQuinticTraj(float t0, float t1, float t, float p0, float p1, float v0, float v1, float a0, float a1);
    float getPos();
    float getVel();
    float getAcc();

VectorXf x;
    
private:
    float p_ = 0;
    float v_ = 0;
    float a_ = 0;
};

#endif