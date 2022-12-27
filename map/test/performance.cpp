#include "gtest/gtest.h"
#include "map/map.hpp"
#include <chrono>
#include <iostream>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

TEST(Map, Performance){
    Room room(10,10);
    std::cout << "room" <<std::endl;
    random_obstacles(&room, 6, 20);
    std::cout << "obstacles" <<std::endl;
    RoadMap map(room);
    map.constructRoadMap(80, 4, 0.5, 500);
    std::cout << "road map" <<std::endl;
    auto start = high_resolution_clock::now();

    compute_roadmap_dubins(&map);

    std::cout<<map.get_curves().size()<<std::endl;
    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<ms_double.count()<<"ms"<<std::endl;

    EXPECT_TRUE(true);
}