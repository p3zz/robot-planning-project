#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <utils/utils.h>


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

// arc (portion of dubin curve)
class DubinArc {
    public: 
        DubinPoint source;
        double length;
        double k;

        DubinArc(DubinPoint source, double length, double k):source{source},length{length},k{k}{}

        DubinPoint get_dest(){
            double x = this->source.x + this->length * sinc(this->k * this->length / 2.0) * cos(this->source.th + this->k * this->length / 2);
            double y = this->source.y + this->length * sinc(this->k * this->length / 2.0) * sin(this->source.th + this->k * this->length / 2);
            double th = mod2pi(this->source.th + this->k * this->length);
            return DubinPoint(x,y,th);
        }
};

class DubinCurve {
    public:
        DubinArc arc1;
        DubinArc arc2;
        DubinArc arc3;

        DubinCurve(DubinArc arc1, DubinArc arc2, DubinArc arc3):arc1{arc1},arc2{arc2},arc3{arc3}{}

        double get_length(){
            return arc1.length + arc2.length + arc3.length; 
        }
};

struct primitive{ 
    bool ok;
    double sc_s1_c,sc_s2_c,sc_s3_c;
};

bool dubins_shortest_path(DubinPoint p_start, DubinPoint p_end);
Point getTraj(int x);
int getLenTraj();

#endif