#ifndef SHAPES_H
#define SHAPES_H

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>

#include "dubins/dubins.hpp"

class Point2D{
    public:
        double x,y;

        Point2D(){}
        Point2D(double x, double y):x{x},y{y}{}

        double distance_from(Point2D p);
        Point2D traslate(double offset_x, double offset_y);
        friend bool operator== (const Point2D& first, const Point2D& second) {return first.x == second.x && first.y == second.y;}
        friend std::ostream& operator<<(std::ostream& os, const Point2D& p){return os<<"("<<p.x<<";"<<p.y<<")";}
};

class Segment {
    public:
        Point2D node1, node2;
        Segment(Point2D node1, Point2D node2):node1{node1},node2{node2}{};

        bool contains(Point2D p);
        Point2D get_interceptor(double t);
        Segment traslate(double offset);
        double get_slope();
};

class Circle{
    public:
        // radius
        double r;
        // center
        Point2D c;

        Circle(Point2D c, double r):r{r},c(c){}

        bool contains(Point2D p);
        double get_angle(Point2D p);
};

class Polygon{
    public:
        std::vector<Point2D> vertexes;

        Polygon():vertexes{}{}
        Polygon(std::vector<Point2D> vertexes):vertexes{vertexes}{}

        std::vector<Segment> get_sides();
        bool contains(Point2D p);
        void add_v(Point2D v){vertexes.push_back(v);}
        Point2D get_v(int index){return vertexes.at(index);}
        int get_size(){return vertexes.size();}
};

Circle get_circle(Point2D p1, Point2D p2, Point2D p3);
bool intersect(Segment s1, Segment s2);
bool intersect(Circle c, Segment s, Point2D start, Point2D end);
bool intersect(Polygon p, Segment s);
bool intersect(DubinCurve c, Polygon p);
bool intersect(DubinCurve c, Polygon p, int n);
double distance(Point2D p1, Point2D p2);

#endif
