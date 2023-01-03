#ifndef DECISIONS_H
#define DECISIONS_H

#include <iostream>
#include <utils/utils.h>
#include <fstream>
#include "dubins/dubins.hpp"
#include "map/map.hpp"

class PayoffMatrix
{
    public:
        RoadMap map;
        Point2D pursuer, evader;
        PayoffMatrix(RoadMap map):map{map}{}
        void computeMove(DubinPoint pursuer, DubinPoint evader, DubinPoint& move_pursuer, DubinPoint& move_evader); //depth=2
};

Point2D getNearestNode(Point2D position, std::vector<Point2D> nodes);
void simulate(Point2D pursuer_pos, Point2D evader_pos, PayoffMatrix decision);

#endif