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

    std::cout << "L1:" << L1 << std::endl;
    std::cout << "setpointL:" << (L1/circumference) * gearRatio << std::endl;
    std::cout << "setpointR:" << (L2/circumference) * gearRatio << std::endl;

    setpointL = (L1/circumference) * gearRatio;
    setpointR = (L2/circumference) * gearRatio;
}