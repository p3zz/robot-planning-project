#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>

#define SEED 0 //if 0 set random seed
#define ROBOT_CIRCLE 0.3  //30 cm diameter of robot occupation
#define ROBOT_VELOCITY 1.0  //1 m/s velocity of robot

void init_seed(int seed); //seed=0 equal to random seed
int get_seed();
double mod2pi(double angle);
double modpi(double angle);
double sinc(double t);

#endif