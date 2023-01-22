#include "gtest/gtest.h"
#include "dubins/dubins.hpp"
#include <chrono>
#include <iostream>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

auto p_src = DubinPoint(0, 0, M_PI/4);
auto p_dst = DubinPoint(3, 4, M_PI/2);
auto curve = dubins_shortest_path(p_src, p_dst, 3);

Polygon p({ Point2D(4.5, 7.5), Point2D(5, 7.5), Point2D(5.49, 7.34), Point2D(6, 7),
            Point2D(6.28, 0.5), Point2D(6.2, 6), Point2D(5.74, 5.53), Point2D(4.99, 5.41),
            Point2D(4.28, 5.51), Point2D(3.63, 5.97), Point2D(3.5, 6.5), Point2D(3.5, 7),
            Point2D(3.77, 7.49)} );


TEST(DubinCurve, IntersectPolygon){
    auto start = high_resolution_clock::now();
    intersect(curve, p);
    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<ms_double.count()<<"ms"<<std::endl;
}

TEST(DubinCurve, IntersectPolygonReduced_100Points){
    auto start = high_resolution_clock::now();
    intersect(curve, p, 100);
    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<ms_double.count()<<"ms"<<std::endl;
}

TEST(DubinCurve, IntersectPolygonReduced_50Points){
    auto start = high_resolution_clock::now();
    intersect(curve, p, 50);
    auto end = high_resolution_clock::now();

    duration<double, std::milli> ms_double = end - start;
    std::cout<<ms_double.count()<<"ms"<<std::endl;
}