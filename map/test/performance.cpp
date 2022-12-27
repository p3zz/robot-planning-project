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
    random_obstacles(&room, 8, 20);
    std::cout << "obstacles" <<std::endl;
    RoadMap map(room);
    map.constructRoadMap(100, 4, 0.5, 500);
    std::cout << "road map" <<std::endl;
    auto start = high_resolution_clock::now();

    for(auto link: map.getLinks()){
        // std::cout << link.node1 << link.node2 <<std::endl;
        double step = M_PI * 0.25;
        for(double th_src = 0; th_src < 2 * M_PI; th_src += step){
            DubinPoint src(link.node1.x, link.node1.y, th_src);
            for(double th_dst = 0; th_dst < 2 * M_PI; th_dst += step){
                // std::cout << th_src << "\t" << th_dst <<std::endl;
                DubinPoint dst(link.node2.x, link.node2.y, th_dst);
                auto curves = dubin_curves(src, dst);
                for(auto curve: curves){
                    bool inter = false;
                    for(auto ob: room.get_obstacles()){
                        if(intersect(curve, ob)){
                            inter = true;
                            break;
                        }
                    }
                    if(!inter){
                        // std::cout << "curve found" <<std::endl;                    
                        break;
                    }
                }
            }
        }
    }

    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<ms_double.count()<<"ms"<<std::endl;

    EXPECT_TRUE(true);
}