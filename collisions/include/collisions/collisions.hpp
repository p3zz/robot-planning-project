#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <cmath>
#include <vector>

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
        bool intersect(Segment s);
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

        // check if this circle intersect a given segment into a bound [min, max]
        // this bound simulates an arc described by its ends
        bool intersect(Segment s, Point2D min, Point2D max);
        bool contains(Point2D p);
        double get_angle(Point2D p);
};

#endif
