#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <cstdlib>

#define ROBOT_CIRCLE 0.3  //30 cm diameter of robot occupation
#define ROBOT_VELOCITY 1.0  //1 m/s velocity of robot

double mod2pi(double angle);
double modpi(double angle);
double sinc(double t);

#endif