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
        DubinPoint(Point2D p, double th):x{p.x},y{p.y},th{th}{}

        Point2D get_point(){return Point2D(x,y);}

        friend bool operator== (const DubinPoint& first, const DubinPoint& second) {return first.x == second.x && first.y == second.y && first.th <= second.th*1.03 && first.th >= second.th*0.97;}
};
string operator + (string s, DubinPoint& dp);

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
        std::vector<DubinPoint> to_points_homogeneous(double sens);
};
string operator + (string s, DubinArc& da);

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
        std::vector<DubinPoint> to_points_homogeneous(double sens);
};
string operator + (string s, DubinCurve& dc);

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
        std::string to_json(double precision);
};

struct primitive{ 
    bool ok;
    double curve1_len, curve2_len, curve3_len;
};

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end, const double curvature);
std::vector<DubinCurve> dubin_curves(DubinPoint p_start, DubinPoint p_end, const double curvature);
bool intersect(DubinCurve c, Polygon p);
bool intersect(DubinCurve c, Segment2D s);
bool intersect(DubinCurve c, Polygon p, int n);
bool intersect_sides(DubinCurve c, Polygon p);

#endif