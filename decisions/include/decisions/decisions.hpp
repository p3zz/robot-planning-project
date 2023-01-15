#ifndef DECISIONS_H
#define DECISIONS_H

#include <iostream>
#include <utils/utils.h>
#include <fstream>
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
};
string operator + (string s, Path& path);


class PayoffMatrix
{
    public:
        RoadMap map;
        Point2D pursuer, evader;
        PayoffMatrix(RoadMap map):map{map}{}
        bool compute_move(DubinPoint pursuer, DubinPoint evader, Path& path_pursuer, Path& path_evader); //depth=2
};

std::string get_path_json(Path& path_pursuer, Path& path_evader, double precision);

#endif