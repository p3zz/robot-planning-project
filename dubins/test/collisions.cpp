#include "gtest/gtest.h"
#include "dubins/dubins.hpp"

TEST(DubinCurve, PolygonCollision1){
    DubinPoint src(0.0, 0.0, M_PI / 4);
    DubinPoint dst(5.0, -2.0, M_PI);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(3,1), Point2D(4,1), Point2D(4.53,0), Point2D(4,-1), Point2D(2,-1)});
    EXPECT_TRUE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision2){
    DubinPoint src(0.0, 0.0, M_PI / 4);
    DubinPoint dst(5.0, -2.0, M_PI);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(2,-1), Point2D(3,-1), Point2D(3,-1.5), Point2D(2,-2), Point2D(2,-2)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision3){
    DubinPoint src(0.0, 0.0, M_PI / 4);
    DubinPoint dst(5.0, -2.0, M_PI);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(2,-1), Point2D(3,-1), Point2D(3,-1.5), Point2D(2.56,-0.29), Point2D(1.15,0.24)});
    EXPECT_TRUE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision4){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(-2,2), Point2D(-1,1), Point2D(-1,4)});
    EXPECT_TRUE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision5){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(-2,2), Point2D(-1,3), Point2D(-1,4)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision6){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst);

    Polygon p({Point2D(-3,3), Point2D(2,3), Point2D(2,-2), Point2D(-3,-2)});
    EXPECT_TRUE(intersect(curve, p));
}