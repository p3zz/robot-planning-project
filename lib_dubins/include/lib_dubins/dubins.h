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

class Point //structure of point: set all 5 parameters for every point, then len_traj (length of non-empty part of array), when you draw a trajectory!
{
    public: 
        double x;
        double y;
        double th;
        double v;
        double w;

        Point(double x, double y, double th, double v, double w):x{x},y{y},th{th},v{v},w{w}{}
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

        DubinPoint():x{0},y{0},th{0}{}
        DubinPoint(double x, double y, double th):x{x},y{y},th{th}{}

};

// arc (portion of dubin curve)
class DubinArc {
    public: 
        DubinPoint source;
        double length;
        double k;

        DubinArc():source{DubinPoint()}, length{0}, k{0}{};
        DubinArc(DubinPoint source, double length, double k):source{source},length{length},k{k}{}

        DubinPoint get_dest(){
            double x = this->source.x + this->length * sinc(this->k * this->length / 2.0) * cos(this->source.th + this->k * this->length / 2);
            double y = this->source.y + this->length * sinc(this->k * this->length / 2.0) * sin(this->source.th + this->k * this->length / 2);
            double th = mod2pi(this->source.th + this->k * this->length);
            return DubinPoint(x,y,th);
        }
};

std::vector<Point> arc_to_points(DubinArc arc, int n_points, double velocity, bool acc, bool dec);

class DubinCurve {
    public:
        std::vector<DubinArc> arcs;

        DubinCurve(){
            arcs = {DubinArc(), DubinArc(), DubinArc()};
        }
        DubinCurve(DubinArc arc1, DubinArc arc2, DubinArc arc3){
            arcs = {arc1, arc2, arc3};
        }

        double get_length(){
            double length = 0;
            for(auto arc:arcs) length+=arc.length;
            return length; 
        }

        std::vector<Point> get_trajectory(){

            std::vector<Point> points;

            double const length_threshold = 2.0;            

            //1st arc
            double velocity = VELOCITY_AVG;
            double const velocity_straight = velocity + 0.2;
            double const velocity_straight_overlength = velocity + 0.5;

            for(DubinArc arc:arcs){
                velocity = VELOCITY_AVG;

                // if the arc is straight, go faster
                if(int(arc.k / CURV_MAX) == S){
                    velocity = arc.length > length_threshold? velocity_straight_overlength : velocity_straight;
                }

                int arc_points_n = int((arc.length / get_length()) * PRECISION_TRAJ);
                std::vector<Point> arc_points = arc_to_points(arc, arc_points_n, velocity, true, false);
                points.insert(points.end(), arc_points.begin(), arc_points.end());
            }

            return points;
        }
};

struct primitive{ 
    bool ok;
    double sc_s1_c,sc_s2_c,sc_s3_c;
};

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end);

#endif