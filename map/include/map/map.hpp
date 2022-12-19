#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>

#include "shapes/shapes.hpp"
#include "utils/utils.h"

#define KNN_MAX 30
#define POINT_COORD_MAX 999999

class Room
{
    private: 
        double h;
        double w;
        std::vector<Polygon> obstacles;
    public:
        Room(double h, double w):h{h},w{w}{}
        void addObstacle(Polygon o){obstacles.push_back(o);}
        Polygon getObstacle(int index){return obstacles.at(index);}
        int getNumObstacles(){return obstacles.size();}
        int getHeight(){return h;}
        int getWidth(){return w;}

};

class RoadMap
{
    private: 
        Room r;
        std::vector<Point2D> nodes;
        std::vector<Segment>  links;
    public:
        RoadMap(Room r):r{r}{}
        //PRM ROADMAP
        bool constructRoadMap(int points, int knn, double k_distance_init, double tms_max); //k_distance_init initialize k_distance between 0.1 (inhomogeneus, very easy) and 1 (max homogeneus, very hard) which provides an homogeneus map, then decrease exponentially in time to reach 10% at time tms_max
        std::vector<Point2D> getNodes(){return nodes;}
        std::vector<Segment> getLinks(){return links;}
        std::string getJson();
};



#endif