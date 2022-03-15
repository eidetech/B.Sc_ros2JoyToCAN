#include "ik.h"

IK::IK()
{

}

IK::~IK()
{

}

void IK::setOffsets(float xA, float zA, float xB, float zB)
{
    xA_ = xA;
    zA_ = zA;
    xB_ = xB;
    zB_ = zB;
}

std::vector<float> IK::getOffsetVectors()
{
    std::vector<float> offSetVectors;

    float d1_x = x_path_pos - xA; // x [m] component of distance vector from origo to pulley A
    float d1_z = z_path_pos - zA; // z [m] component of distance vector from origo to pulley A

    float d2_x = x_path_pos - xB; // x [m] component of distance vector from origo to pulley B
    float d2_z = z_path_pos - zB; // z [m] component of distance vector from origo to pulley B

    offSetVectors.push_back(d1_x);
    offSetVectors.push_back(d1_z);
    offSetVectors.push_back(d2_x);
    offSetVectors.push_back(d2_z);

    return offSetVectors;
}