#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <utils/utils.h>
#include <shapes/shapes.hpp>
#include <map>


#define DEBUG_MOBILE true

#define MAX_LENGTH_TRAJ 500.0 // 100 meter max
// #define PRECISION_TRAJ 100.0  // cms precision
#define CURV_MAX 1.0          // max curvature

#define DOUBLE_MAX 1.79769e+308

enum CurveType{
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL,
    None = -1
};

enum ArcType{
    Straight = 0,
    Left = 1,
    Right = -1
};

// point (start/end of a dubin arc)
class DubinPoint{
    public: 
        double x;
        double y;
        double th;

        DubinPoint():x{0},y{0},th{0}{}
        DubinPoint(double x, double y, double th):x{x},y{y},th{th}{}

        friend bool operator== (const DubinPoint& first, const DubinPoint& second) {return first.x == second.x && first.y == second.y && first.th == second.th;}
};

// arc (portion of dubin curve)
class DubinArc {
    public: 
        DubinPoint source;
        double length;
        double k;

        DubinArc():source{DubinPoint()}, length{0}, k{0}{};
        DubinArc(DubinPoint source, double length, double k):source{source},length{length},k{k}{}

        DubinPoint get_dest();
        std::vector<DubinPoint> to_points(int n_points);
};

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

        std::vector<DubinPoint> to_points(int n);
};

class DubinLink{
    private:
        DubinPoint src, dst;
        DubinCurve curve;
        bool empty;
    public:
        DubinLink():empty{true}{}
        DubinLink(DubinPoint src, DubinPoint dst, DubinCurve curve):src{src}, dst{dst}, curve{curve}, empty{false}{}
        DubinPoint get_src(){return src;}
        DubinPoint get_dst(){return dst;}
        DubinCurve get_curve(){return curve;}
        bool is_empty(){return empty;}
};

struct primitive{ 
    bool ok;
    double curve1_len, curve2_len, curve3_len;
};

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end);
std::vector<DubinCurve> dubin_curves(DubinPoint p_start, DubinPoint p_end);
bool intersect(DubinCurve c, Polygon p);
bool intersect(DubinCurve c, Polygon p, int n);

#endif