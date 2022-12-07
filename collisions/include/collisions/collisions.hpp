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

class Segment{
    public:
        Point2D src,dst;

        Segment(Point2D src, Point2D dst):src{src},dst{dst}{}


        bool intersect(Segment s);
        bool contains(Point2D p);
        Point2D get_interceptor(double t);

};

class Circle{
    public:
        double r;
        Point2D c;

        Circle(Point2D c, double r):r{r},c(c){}

        std::vector<Point2D> intersect(Segment s);
        double get_angle(Point2D p);
};

class Polygon{
    private:
        std::vector<Point2D> vertexes;
    public:
        Polygon(){}        
        void addVertex(Point2D v){vertexes.push_back(v);}
        Point2D getVertex(int index){return vertexes.at(index);}
        int getNumVertexes(){return vertexes.size();}

};

#endif
