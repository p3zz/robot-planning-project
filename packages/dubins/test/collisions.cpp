#include "gtest/gtest.h"
#include "dubins/dubins.hpp"

TEST(DubinCurve, PolygonCollision1){
    DubinPoint src(0.0, 0.0, M_PI / 4);
    DubinPoint dst(5.0, -2.0, M_PI);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(3,1), Point2D(4,1), Point2D(4.53,0), Point2D(4,-1), Point2D(2,-1)});
    EXPECT_TRUE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision2){
    DubinPoint src(0.0, 0.0, M_PI / 4);
    DubinPoint dst(5.0, -2.0, M_PI);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(2,-1), Point2D(3,-1), Point2D(3,-1.5), Point2D(2,-2)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, SegmentCollision1){
    DubinPoint src(1.39, -3.77, M_PI * 0.75);
    DubinPoint dst(2.0, -4.58, M_PI * 0.5);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1, -2.5), Point2D(1, -5.5));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision2){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1, -2.5), Point2D(1, -5.5));
    EXPECT_FALSE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision3){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(0, -4), Point2D(4, -4));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision4){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1.5, -6), Point2D(1.5, -2.5));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision5){
    DubinPoint src(1.39, -3.77, M_PI * 0.75);
    DubinPoint dst(2.0, -4.58, M_PI * 0.5);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1, -5.5), Point2D(1, -2.5));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision6){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1, -5.5), Point2D(1, -2.5));
    EXPECT_FALSE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision7){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(4, -4), Point2D(0, -4));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, SegmentCollision8){
    DubinPoint src(1.39, -3.77, M_PI * 0.25);
    DubinPoint dst(2.0, -4.58, M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Segment2D s(Point2D(1.5, -6), Point2D(1.5, -2.5));
    EXPECT_TRUE(intersect(curve, s));
}

TEST(DubinCurve, PolygonCollision4){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(-2,2), Point2D(-1,4), Point2D(-1,1)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision5){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst, 2);

    Polygon p({Point2D(-2,2), Point2D(-1,4), Point2D(-1,3)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision6){
    DubinPoint src(1, 1, M_PI / 2);
    DubinPoint dst(0, 0,  M_PI / 2);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(-3,3), Point2D(2,3), Point2D(2,-2), Point2D(-3,-2)});
    EXPECT_TRUE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision7){
    DubinPoint src(0, 0, M_PI * 0.25);
    DubinPoint dst(0, -1,  M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(0.5, 2), Point2D(1.5, 2), Point2D(1.5, 1), Point2D(0.5, 1)});
    EXPECT_FALSE(intersect(curve, p));
}

TEST(DubinCurve, PolygonCollision8){
    DubinPoint src(0, 0, M_PI * 0.25);
    DubinPoint dst(0, -1,  M_PI * 0.75);
    auto curve = dubins_shortest_path(src, dst, 2);
    
    Polygon p({Point2D(1, 1), Point2D(2, 1), Point2D(1, 0)});
    EXPECT_FALSE(intersect(curve, p));
}