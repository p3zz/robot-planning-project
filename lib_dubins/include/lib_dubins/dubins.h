#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>


#define DEBUG_MOBILE true

#define S 0
#define L 1
#define R -1

#define MAX_LENGTH_TRAJ 500.0 // 100 meter max
#define PRECISION_TRAJ 100.0  // cms precision
#define DECELERATION 0.1      // 10 cms of deceleration
#define ACCELERATION 0.1      // 10 cms of acceleration
#define CURV_MAX 1.0          // max curvature
#define VELOCITY_AVG 0.5      // main velocity

#define PI 3.141592654
#define DOUBLE_MAX 1.79769e+308

struct Point //structure of point: set all 5 parameters for every point, then len_traj (length of non-empty part of array), when you draw a trajectory!
{
    double x;
    double y;
    double th;
    double v;
    double w;
};

enum CurveType{
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};

// point (start/end of a dubin arc)
class DubinPoint{
    public: 
        double x;
        double y;
        double th;
         
        DubinPoint(double x, double y, double th):x{x},y{y},th{th}{}

};

bool dubins_shortest_path(DubinPoint p_start, DubinPoint p_end);
Point getTraj(int x);
int getLenTraj();
double mod2pi(double ang);
double sinc(double t);

#endif