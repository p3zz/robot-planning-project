#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>

#define ROBOT_CIRCLE 0.35 //m radius of robot occupation
#define ROBOT_VELOCITY 1.0  //m/s velocity of robot

class Seed
{
    private:
        static int seed;
    public:
        static void init_seed(int seed); //seed=0 equal to random seed
        static int get_seed();
};

double mod2pi(double angle);
double modpi(double angle);
double sinc(double t);

#endif