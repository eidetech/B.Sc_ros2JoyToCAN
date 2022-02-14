#include "math.h"
#include <stdint.h>
#include <iostream>
#include <stdio.h>

// Calculate PI
#define PI 4*atan(1)

class KinematicsCalculations
{
private:
    
public:
    KinematicsCalculations();
    ~KinematicsCalculations();
    void calculate();

    // Kinematic variables and constants
    float setpointL = 0;
    float setpointR = 0;
    float px = 0;
    float pz = 0;
    float L1_start = 50; // Length of left string mm
    float L2_start = 2820 - 50; // Length of right string mm
    float L1 = 0;
    float L2 = 0;
    const float c = 2820; // Length of top aluminium bar mm
    const float r = 45; // Radius of wire wheel in mm
    const float circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
    const float gearRatio = 10.;
    const float encoderCountsPerRev = 2400.*gearRatio; // 2400 encoder count per revolution, and 10:1 gear reduction
    const float resolution = circumference/encoderCountsPerRev;
};

