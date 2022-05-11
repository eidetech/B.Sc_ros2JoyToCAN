#include "quintic.h"

Quintic::Quintic() : matA(6, 6), vecB(6), c(6)
{

}

Quintic::~Quintic()
{

}

// Function for calculating trajectory with quintic polynomial (per axis)
/*      Inputs:
        t0  ==> [s]     time at initial point
        t1  ==> [s]     time at final point
        t  ==>  [s]     current time
        p0  ==> [m]     position of initial point
        p1  ==> [m]     position of final point
        v0  ==> [m/s]   velocity at initial point
        v1  ==> [m/s]   velocity at final point
        a0  ==> [m/s^2] acceleration at initial point
        a1  ==> [m/s^2] acceleration at final point

        Outputs (via getters):
        p   ==> [m]     array of position points
        v   ==> [m/s]   array of velocity points
        a   ==> [m/s^2] array of acceleration points
        t   ==> time array used to generate trajectory
*/
void Quintic::calcQuinticTraj(float t0, float t1, float t, float p0, float p1, float v0, float v1, float a0, float a1)
{
    // Time matrix
    matA << 1, t0, pow(t0,2), pow(t0,3),  pow(t0,4),   pow(t0,5),
            1, t1, pow(t1,2), pow(t1,3),  pow(t1,4),   pow(t1,5),
            0, 1,    2*t0,   3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
            0, 1,    2*t1,   3*pow(t1,2), 4*pow(t1,3), 5*pow(t1,4),
            0, 0,     2,       6*t0,     12*pow(t0,2), 20*pow(t0,3),
            0, 0,     2,       6*t1,     12*pow(t1,2), 20*pow(t1,3);

    // Trajectory constraints for position, velocity and acceleration
    vecB << p0,
            p1,
            v0,
            v1,
            a0,
            a1;

    // Solving the six coefficients for the fifth order polynomial
    // A*c = b  ==>   c = A^(-1)*b
    // Using colPivHouseholderQr for speed and precision
    c = matA.colPivHouseholderQr().solve(vecB);

    // Calculate position (p_), velocity (v_) and acceleration (a_)
    p_ = c(0) + c(1)*t + c(2)*pow(t,2) + c(3)*pow(t,3) + c(4)*pow(t,4) + c(5)*pow(t,5); // position
    v_ = c(1) + 2*c(2)*t + 3*c(3)*pow(t,2) + 4*c(4)*pow(t,3) + 5*c(5)*pow(t,4);         // velocity
    a_ = 2*c(2) + 6*c(3)*t + 12*c(4)*pow(t,2) + 20*c(5)*pow(t,3);                       // acceleration
}

// Get the position variable
float Quintic::getPos(){return p_;}

// Get the velocity variable
float Quintic::getVel(){return v_;}

// Get the acceleration variable
float Quintic::getAcc(){return a_;}