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
#include "dubins/dubins.hpp"

#define KNN_MAX 10
#define POINT_COORD_MAX 999999
#define MAX_CURVATURE 5

class Room
{
    private: 
        Polygon dimensions;
        Polygon dimensions_deflated;
        double approx_h, approx_w, approx_area, off_x, off_y;
        std::vector<Polygon> obstacles;
        std::vector<Polygon> obstacles_inflated;
        std::vector<Point2D> exits;
        std::vector<Point2D> exits_inflated;
    public:
        Room(Polygon dimensions);
        void add_obstacle(Polygon o){obstacles.push_back(o); obstacles_inflated.push_back(inflate(o, ROBOT_CIRCLE));}
        Polygon get_obstacle(int index){return obstacles.at(index);}
        std::vector<Polygon> get_obstacles(){return obstacles;};
        Polygon get_inflated_obstacle(int index){return obstacles_inflated.at(index);}
        std::vector<Polygon> get_inflated_obstacles(){return obstacles_inflated;};
        int get_num_obstacles(){return obstacles.size();}
        void add_exit(Point2D exit);
        Point2D get_exit(int index, bool inflated){if(inflated) return exits_inflated.at(index); return exits.at(index);}
        int get_num_exits(){return exits.size();}
        Polygon get_dimensions(bool deflated){if(deflated) return dimensions_deflated; return dimensions;}
        double get_approx_height(){return approx_h;}
        double get_approx_width(){return approx_w;}
        double get_approx_area(){return approx_area;}
        double get_offset_x(){return off_x;}
        double get_offset_y(){return off_y;}

};

class RoadMap
{
    private:
        Room r;
        std::vector<Point2D> nodes;
        std::vector<Segment2D> links;
        std::vector<DubinLink> dubin_links;
    public:
        RoadMap(Room r):r{r}{}
        //PRM ROADMAP
        bool construct_roadmap(int points, int knn, double k_distance_init, double tms_max, Point2D p_pos, Point2D e_pos); //k_distance_init initialize k_distance between 0.1 (inhomogeneus, very easy) and 1 (max homogeneus, very hard) which provides an homogeneus map, then decrease exponentially in time to reach 10% at time tms_max
        std::vector<Point2D> get_nodes(){return nodes;}
        std::vector<Segment2D> get_links(){return links;}
        int get_node_index(Point2D node);
        std::vector<DubinLink> get_curves(){return dubin_links;}
        void get_attached_nodes(Point2D node, std::vector<Point2D> *attached_nodes);
        DubinLink get_dubin_link(DubinPoint dp1, DubinPoint dp2);
        Room& get_room(){return r;}
        std::string get_json();
};

void random_obstacles_side(Room* room, int num_obstacles, const int max_side);
void random_obstacles_vertexes(Room* room, int num_obstacles, int vertexes_n);

#endif