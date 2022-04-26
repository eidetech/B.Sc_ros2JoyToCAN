#include "ik.h"

IK::IK()
{

}

IK::~IK()
{

}

// Calculate the inverse kinematics [x,z] -> [q1,q2]
void IK::calc(float x, float z, float x_t, float z_t)
{
    float d1_x = x - xA_;  // x [m] component of right triangle
    float d1_z = z - zA_; // z [m] component of right triangle

    float d2_x = x - xB_;  // x [m] component of right triangle
    float d2_z = z - zB_; // z [m] component of right triangle

    float L_1 = sqrt(pow(d1_x,2) + pow(d1_z,2)); // [m] actual length of wire L1 (from pulley A -> TP)
    float theta_1 = atan2(d1_z,d1_x);            // [rad] angle between horizontal line between pulleys and L1
    q1_ = (L_wire-L_1)/(R*2*PI)*gear_ratio;      // [rev] angular position of motor M1

    // angular velocity of motor M1 [rev/s]
    q1_t_ = (z_t + (x_t*cos(theta_1)) / (sin(theta_1))) / (sin(theta_1) + (pow(cos(theta_1),2)) / (sin(theta_1)))/(R*2*PI)*gear_ratio;

    float L_2 = sqrt(pow(d2_x,2) + pow(d2_z,2)); // [m] actual length of wire L2 (from pulley B -> TP)
    float theta_2 = atan2(d2_z,d2_x);            // [rad] angle between horizontal line between pulleys and L2
    q2_ = (L_wire-L_2)/(R*2*PI)*gear_ratio;      // [rev] angular position of motor M0

    // angular velocity of motor M0 [rev/s]
    q2_t_ = (z_t + (x_t*cos(theta_2)) / (sin(theta_2))) / (sin(theta_2) + (pow(cos(theta_2),2)) / (sin(theta_2)))/(R*2*PI)*gear_ratio;
}

// Set offsets in x and z direction from origo
void IK::setOffsets(float xA, float zA, float xB, float zB)
{
    xA_ = xA;
    zA_ = zA;
    xB_ = xB;
    zB_ = zB;
}

// Get the angular position of q1
float IK::getAngPos_q1(){return q1_;}

// Get the angular velocity of q1
float IK::getAngVel_q1(){return q1_t_;}

// Get the angular position of q2
float IK::getAngPos_q2(){return q2_;}

// Get the angular velocity of q2
float IK::getAngVel_q2(){return q2_t_;}