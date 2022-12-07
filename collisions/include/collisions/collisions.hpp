#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <cmath>
#include <vector>

class Point{
    public:
        double x;
        double y;

        Point(double x, double y):x{x},y{y}{}
};

class Segment{
    public:
        Point p;
        Point q;

        Segment(Point p, Point q):p{p},q{q}{}


        bool intersect(Segment s);
        bool contains(Point p);
        Point get_interceptor(double t);

};


class Circle{
    public:
        double radius;
        Point center;

        Circle(Point c, double r):radius{r},center(c){}

        std::vector<Point> intersect(Segment s);
        double get_angle(Point p);
};

#endif
