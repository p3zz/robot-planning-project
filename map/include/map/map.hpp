#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iostream>
#include <random>

#include "shapes/shapes.hpp"
#include "utils/utils.h"

#define KNN_MAX 10
#define POINT_COORD_MAX 999999

class Room
{
    private: 
        double h;
        double w;
        std::vector<Polygon> obstacles;
        std::vector<Polygon> obstacles_inflated;
        std::vector<Point2D> exits;
    public:
        Room(double h, double w):h{h},w{w}{}
        void addObstacle(Polygon o){obstacles.push_back(o); obstacles_inflated.push_back(inflate(o, ROBOT_CIRCLE*1.1));}
        Polygon getObstacle(int index){return obstacles.at(index);}
        std::vector<Polygon> get_obstacles(){return obstacles;};
        Polygon getInflatedObstacle(int index){return obstacles_inflated.at(index);}
        int getNumObstacles(){return obstacles.size();}
        void addExit(Point2D exit){exits.push_back(exit);}
        Point2D getExit(int index){return exits.at(index);}
        int getNumExits(){return exits.size();}
        int getHeight(){return h;}
        int getWidth(){return w;}

};

class RoadMap
{
    private: 
        Room r;
        std::vector<Point2D> nodes;
        std::vector<Segment> links;
    public:
        RoadMap(Room r):r{r}{}
        //PRM ROADMAP
        bool constructRoadMap(int points, int knn, double k_distance_init, double tms_max); //k_distance_init initialize k_distance between 0.1 (inhomogeneus, very easy) and 1 (max homogeneus, very hard) which provides an homogeneus map, then decrease exponentially in time to reach 10% at time tms_max
        std::vector<Point2D> getNodes(){return nodes;}
        std::vector<Segment> getLinks(){return links;}
        void getAttachedNodes(Point2D node, std::vector<Point2D> *attached_nodes);
        Room& getRoom(){return r;}
        std::string getJson();
};

void randomObstacles(Room* room, int num_obstacles, const int max_side);
void random_obstacles(Room* room, int num_obstacles, int vertexes_n);

#endif