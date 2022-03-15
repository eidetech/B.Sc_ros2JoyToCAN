#include "quintic.h"

Quintic::Quintic() : x(6)
{

}

Quintic::~Quintic()
{

}

void Quintic::calcQuinticTraj(float t0, float t1, float t, float p0, float p1, float v0, float v1, float a0, float a1)
{
    MatrixXf matA(6, 6);
    VectorXf vecB(6);
    matA << 1, t0, pow(t0,2), pow(t0,3),  pow(t0,4),   pow(t0,5),
            1, t1, pow(t1,2), pow(t1,3),  pow(t1,4),   pow(t1,5),
            0, 1,    2*t0,   3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
            0, 1,    2*t1,   3*pow(t1,2), 4*pow(t1,3), 5*pow(t1,4),
            0, 0,     2,       6*t0,     12*pow(t0,2), 20*pow(t0,3),
            0, 0,     2,       6*t1,     12*pow(t1,2), 20*pow(t1,3);

    vecB << p0,
            p1,
            v0,
            v1,
            a0,
            a1;

    // solving the six coefficients for the fifth order polynomial
    // A*x = b  ==>   x = A^(-1)*b
    x = matA.colPivHouseholderQr().solve(vecB);

    p_ = x(0) + x(1)*t + x(2)*pow(t,2) + x(3)*pow(t,3) + x(4)*pow(t,4) + x(5)*pow(t,5); // position
    v_ = x(1) + 2*x(2)*t + 3*x(3)*pow(t,2) + 4*x(4)*pow(t,3) + 5*x(5)*pow(t,4);         // velocity
    a_ = 2*x(2) + 6*x(3)*t + 12*x(4)*pow(t,2) + 20*x(5)*pow(t,3);                       // acceleration
}

// Get the position variable
float Quintic::getPos(){return p_;}

// Get the velocity variable
float Quintic::getVel(){return v_;}

// Get the acceleration variable
float Quintic::getAcc(){return a_;}