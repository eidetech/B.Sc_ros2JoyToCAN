#include "kinematicsCalculations.h"

KinematicsCalculations::KinematicsCalculations()
{
}

KinematicsCalculations::~KinematicsCalculations()
{
}

void KinematicsCalculations::calculate()
{
	L1 = sqrt(px*px+pz*pz) - L1_start;
	L2 = sqrt((c-px)*(c-px)+pz*pz) - L2_start;

    std::cout << "L1:" << L1 << " ," << "L2:" << L2 << std::endl;
    std::cout << "setpointL:" << (L1/circumference) * gearRatio << ", " << "setpointR:" << (L2/circumference) * gearRatio << std::endl;

    // Setpoint for how many turns the motors should spin to acheive the calculated x, z coordinates.
    setpointL = (L1/circumference) * gearRatio;
    setpointR = (L2/circumference) * gearRatio;
}