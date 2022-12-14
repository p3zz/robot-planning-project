#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <cmath>
#include <vector>
#include <iostream>

class Point2D{
    public:
        double x,y;

        Point2D(double x, double y):x{x},y{y}{}

        double distance_from(Point2D p);
};

class Segment {
    public:
        Point2D src, dst;
        Segment(Point2D src, Point2D dst):src{src},dst{dst}{};

        bool contains(Point2D p);
        Point2D get_interceptor(double t);
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

        Polygon(std::vector<Point2D> vertexes):vertexes{vertexes}{}

        std::vector<Segment> get_sides();
    
};

Circle get_circle(Point2D p1, Point2D p2, Point2D p3);
bool intersect(Segment s1, Segment s2);
bool intersect(Circle c, Segment s, Point2D start, Point2D end);
bool intersect(Polygon p, Segment s);

#endif
