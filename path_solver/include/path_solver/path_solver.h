#ifndef PATH_SOLVER_H
#define PATH_SOLVER_H

#include <utils/utils.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>

#define KNN_MAX 30

class Point2D
{
    public:
        double x, y;
        Point2D(double x, double y):x{x},y{y}{}
        Point2D(){}
};

class Link
{
    public:
        Point2D src, dest;
        Link(Point2D src, Point2D dest):src{src},dest{dest}{}
        
};

class Obstacle
{
    private:
        std::vector<Point2D> vertexes;
    public:
        Obstacle(){}        
        void addVertex(Point2D v){vertexes.push_back(v);}
        Point2D getVertex(int index){return vertexes.at(index);}
        int getNumVertexes(){return vertexes.size();}
};

class Room
{
    private: 
        double h;
        double w;
        std::vector<Obstacle> obstacles;
    public:
        Room(double h, double w):h{h},w{w}{}
        void addObstacle(Obstacle o){obstacles.push_back(o);}
        Obstacle getObstacle(int index){return obstacles.at(index);}
        int getNumObstacles(){return obstacles.size();}
        int getHeight(){return h;}
        int getWidth(){return w;}

};

class RoadMap
{
    private: 
        Room r;
        std::vector<Point2D> nodes;
        std::vector<Link>  links;
    public:
        RoadMap(Room r):r{r}{}
        //PRM ROADMAP
        bool constructRoadMap(int points, double k_distance, int seconds_max, int knn); //k_distance between 0.1 (inhomogeneus, easy) and 0.5 (homogeneus, hard) provides an homogeneus map 
        std::vector<Point2D> getNodes(){return nodes;}
        std::vector<Link> getLinks(){return links;}
        std::string getJson();
};



#endif