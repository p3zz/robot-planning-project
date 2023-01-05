#ifndef DECISIONS_H
#define DECISIONS_H

#include <iostream>
#include <utils/utils.h>
#include <fstream>
#include <ctime>
#include "dubins/dubins.hpp"
#include "map/map.hpp"

class Path
{
    public:
        DubinPoint p1;
        DubinLink l1;
        DubinPoint p2;
        DubinLink l2;
        Path(){}

        friend std::ostream& operator<<(std::ostream& os, Path& path){return os<<" -"<<path.l1.get_curve()<<"-> "<<path.p1<<" -"<<path.l2.get_curve()<<"-> "<<path.p2;}
};


class PayoffMatrix
{
    public:
        RoadMap map;
        Point2D pursuer, evader;
        PayoffMatrix(RoadMap map):map{map}{}
        bool computeMove(DubinPoint pursuer, DubinPoint evader, Path& path_pursuer, Path& path_evader); //depth=2
};

std::string get_path_json(Path& path_pursuer, Path& path_evader, double precision);

#endif