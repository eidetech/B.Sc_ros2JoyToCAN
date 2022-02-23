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
    bool parked = true;
    float setpointL = 0;
    float setpointR = 0;
    float px = 0;
    float pz = 0;
    float L1_start = 0; // Length of left string mm
    float L2_start = 2820; // Length of right string mm
    float L1 = 0;
    float L2 = 0;
    const float c = 2820; // Length of top aluminium bar mm
    const float r = 75; // Radius of wire wheel in mm
    const float circumference = 2*PI*r; // Circumference of string wheel mm (2*pi*r)
    const float gearRatio = 10.;
    const float encoderCountsPerRev = 8192.*gearRatio; // 8192 encoder count per revolution, and 10:1 gear reduction
    const float resolution = circumference/encoderCountsPerRev;
};

