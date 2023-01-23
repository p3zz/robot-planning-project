#include "gtest/gtest.h"
#include "map/map.hpp"
#include <chrono>
#include <iostream>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

Room room(Polygon({ Point2D(0,0), Point2D(10,0), Point2D(10,10), Point2D(0,10) }));
DubinPoint pursuer(2,8,M_PI*1.75);
DubinPoint evader(8,2,M_PI*0.75);

TEST(Map, Performance1){
    random_obstacles_vertexes(&room, 6, 20);
    RoadMap map(room);
    auto start = high_resolution_clock::now();

    map.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point());

    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<"Execution time: "<<ms_double.count()<<" ms"<<std::endl;
    /*auto links = map.get_links();
    auto curves = map.get_curves();
    EXPECT_EQ((int)links.size(), (int)curves.size());
    for(int i=0;i<(int)links.size();i++){
        std::cout<<links[i].node1<<links[i].node2<<curves[i].size()<<std::endl;
    }*/
}

TEST(Map, Performance2){
    random_obstacles_vertexes(&room, 6, 20);
    RoadMap map(room);
    auto start = high_resolution_clock::now();

    map.construct_roadmap(80, 4, 0.5, 500, pursuer.get_point(), evader.get_point());

    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<"Execution time: "<<ms_double.count()<<" ms"<<std::endl;
    /*auto links = map.get_links();
    auto curves = map.get_curves();
    EXPECT_EQ((int)links.size(), (int)curves.size());
    for(int i=0;i<(int)links.size();i++){
        std::cout<<links[i].node1<<links[i].node2<<curves[i].size()<<std::endl;
    }*/
}

TEST(Map, Performance3){
    random_obstacles_vertexes(&room, 8, 20);
    RoadMap map(room);
    auto start = high_resolution_clock::now();

    map.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point());

    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<"Execution time: "<<ms_double.count()<<" ms"<<std::endl;
    /*auto links = map.get_links();
    auto curves = map.get_curves();
    EXPECT_EQ((int)links.size(), (int)curves.size());
    for(int i=0;i<(int)links.size();i++){
        std::cout<<links[i].node1<<links[i].node2<<curves[i].size()<<std::endl;
    }*/

}

TEST(Map, Performance4){
    random_obstacles_vertexes(&room, 8, 20);
    RoadMap map(room);
    auto start = high_resolution_clock::now();

    map.construct_roadmap(80, 4, 0.5, 500, pursuer.get_point(), evader.get_point());

    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<"Execution time: "<<ms_double.count()<<" ms"<<std::endl;
    /*auto links = map.get_links();
    auto curves = map.get_curves();
    EXPECT_EQ((int)links.size(), (int)curves.size());
    for(int i=0;i<(int)links.size();i++){
        std::cout<<links[i].node1<<links[i].node2<<curves[i].size()<<std::endl;
    }*/

}