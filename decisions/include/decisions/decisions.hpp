#ifndef DECISIONS_H
#define DECISIONS_H

#include <iostream>
#include <utils/utils.h>
#include "dubins/dubins.hpp"
#include "map/map.hpp"

class PayoffMatrix
{
    private:
        RoadMap map;
        Point2D pursuer, evader;
    public:
        PayoffMatrix(RoadMap map):map{map}{}
        Point2D computeMove(Point2D pursuer, Point2D evader); //depth=2
};


#endif