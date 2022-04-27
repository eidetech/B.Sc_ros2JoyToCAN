#include <math.h>

#ifndef IK_H
#define IK_H

#pragma once

#ifndef PI
#define PI 3.14159265359
#endif

class IK
{
public:
    // Constructor
    IK();
    // Destructor
    ~IK();

    // Calculate the inverse kinematics [x,z] -> [q1,q2]
    void calc(float x, float z, float x_t, float z_t);

    // Set offsets in x and z direction from origo
    void setOffsets(float xA, float zA, float xB, float zB);

    // Getter functions
    float getAngPos_q1();
    float getAngVel_q1();
    float getAngPos_q2();
    float getAngVel_q2();

private:    
    float R = 125.5/(2*1000);   // [m] radius of spool
    float L_wire = 19.5*2*PI*R; // [m] total length of wire on spool

    // TODO: Update gear_ratio to new winch system
    float gear_ratio = 10.0;    // ratio of the the gearbox between motor and spool
    float cpr = 8192.0;         // encoder counts per revolution

    float xA_ = 0;
    float zA_ = 0;
    float xB_ = 0;
    float zB_ = 0;

    float q1_ = 0;
    float q2_ = 0;
    float q1_t_ = 0;
    float q2_t_ = 0;
};

#endif