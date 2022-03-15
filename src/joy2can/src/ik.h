#include <vector>

#ifndef IK_H
#define IK_H

#pragma once

class IK
{
public:
    IK();
    ~IK();

    void setOffsets(float xA, float zA, float xB, float zB);
    std::vector<float> getOffsetVectors();

private:
    float L_wire = 8.800; // [m] total length of wire on spool
    
    float R = 0.075; // [m] radius of spool
    float gear_ratio = 10.0; // ratio of the the gearbox between motor and spool
    float cpr = 8192.0; // encoder counts per revolution

    float xA_ = 0;
    float zA_ = 0;
    float xB_ = 0;
    float zB_ = 0;

};

#endif