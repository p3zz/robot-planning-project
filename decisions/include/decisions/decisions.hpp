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

        std::string to_json(double precision);
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

std::string get_pursuer_evader_path_json(Path path_pursuer, Path path_evader, double precision);
std::string get_pursuer_evader_moves_json(std::vector<DubinLink> pursuer_moves, std::vector<DubinLink> evader_moves, double precision);

#endif