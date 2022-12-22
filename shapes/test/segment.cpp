#include "gtest/gtest.h"
#include "shapes/shapes.hpp"

// intersect

// incident
TEST(Segment, IntersectIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(0, 2));
    EXPECT_TRUE(intersect(s1, s2));
}

// not incident
TEST(Segment, IntersectNotIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 10));
    Segment s2(Point2D(1, 0), Point2D(3, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// parallel
TEST(Segment, IntersectParallel){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(3, 0), Point2D(5, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// parallel 2
TEST(Segment, IntersectParallel2){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(4, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// overlap
TEST(Segment, IntersectOverlap){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(1, 1), Point2D(3, 3));
    EXPECT_TRUE(intersect(s1, s2));
}

// contains
TEST(Segment, Contains){
    Segment s(Point2D(0, 0), Point2D(2, 2));
    Point2D p(1,1);
    EXPECT_TRUE(s.contains(p));
}

TEST(Segment, ContainsEnd){
    Segment s(Point2D(0, 0), Point2D(2, 2));
    Point2D p(2,2);
    EXPECT_TRUE(s.contains(p));
}

TEST(Segment, ContainsOutsideSameLine){
    Segment s(Point2D(2, 3), Point2D(3, 4));
    Point2D p(4,5);
    EXPECT_FALSE(s.contains(p));
}

TEST(Segment, ContainsOutsideDifferentLine){
    Segment s(Point2D(2, 3), Point2D(3, 4));
    Point2D p(2,4);
    EXPECT_FALSE(s.contains(p));
}

// traslate

TEST(Segment, Translate1){
    Segment s(Point2D(2, 3), Point2D(3, 4));
    auto s_new = translate(s, 2);
    EXPECT_NEAR(s_new.node1.x, 0.58, 0.1);
    EXPECT_NEAR(s_new.node1.y, 4.42, 0.1);
    EXPECT_NEAR(s_new.node2.x, 1.58, 0.1);
    EXPECT_NEAR(s_new.node2.y, 5.42, 0.1);
}

TEST(Segment, Translate2){
    Segment s(Point2D(1, 4), Point2D(3, 2));
    auto s_new = translate(s, 3);
    EXPECT_NEAR(s_new.node1.x, 3.12, 0.1);
    EXPECT_NEAR(s_new.node1.y, 6.12, 0.1);
    EXPECT_NEAR(s_new.node2.x, 5.12, 0.1);
    EXPECT_NEAR(s_new.node2.y, 4.12, 0.1);
}

TEST(Segment, Translate3){
    Segment s(Point2D(2, 4), Point2D(1, 3));
    auto s_new = translate(s, 2);
    EXPECT_NEAR(s_new.node1.x, 3.41, 0.1);
    EXPECT_NEAR(s_new.node1.y, 2.59, 0.1);
    EXPECT_NEAR(s_new.node2.x, 2.41, 0.1);
    EXPECT_NEAR(s_new.node2.y, 1.59, 0.1);
}

TEST(Segment, Translate4){
    Segment s(Point2D(2, 4), Point2D(1, 5));
    auto s_new = translate(s, 2);
    EXPECT_NEAR(s_new.node1.x, 0.59, 0.1);
    EXPECT_NEAR(s_new.node1.y, 2.59, 0.1);
    EXPECT_NEAR(s_new.node2.x, -0.41, 0.1);
    EXPECT_NEAR(s_new.node2.y, 3.59, 0.1);
}

TEST(Segment, TranslateParallelX1){
    Segment s(Point2D(2, 3), Point2D(3, 3));
    auto s_new = translate(s, 1);
    EXPECT_NEAR(s_new.node1.x, 2, 0.1);
    EXPECT_NEAR(s_new.node1.y, 4, 0.1);
    EXPECT_NEAR(s_new.node2.x, 3, 0.1);
    EXPECT_NEAR(s_new.node2.y, 4, 0.1);
}

TEST(Segment, TranslateParallelX2){
    Segment s(Point2D(2, 3), Point2D(1, 3));
    auto s_new = translate(s, 1);
    EXPECT_NEAR(s_new.node1.x, 2, 0.1);
    EXPECT_NEAR(s_new.node1.y, 2, 0.1);
    EXPECT_NEAR(s_new.node2.x, 1, 0.1);
    EXPECT_NEAR(s_new.node2.y, 2, 0.1);
}

TEST(Segment, TranslateParallelY1){
    Segment s(Point2D(2, 3), Point2D(2, 4));
    auto s_new = translate(s, 1);
    EXPECT_NEAR(s_new.node1.x, 1, 0.1);
    EXPECT_NEAR(s_new.node1.y, 3, 0.1);
    EXPECT_NEAR(s_new.node2.x, 1, 0.1);
    EXPECT_NEAR(s_new.node2.y, 4, 0.1);
}

TEST(Segment, TranslateParallelY2){
    Segment s(Point2D(2, 4), Point2D(2, 3));
    auto s_new = translate(s, 1);
    EXPECT_NEAR(s_new.node1.x, 3, 0.1);
    EXPECT_NEAR(s_new.node1.y, 4, 0.1);
    EXPECT_NEAR(s_new.node2.x, 3, 0.1);
    EXPECT_NEAR(s_new.node2.y, 3, 0.1);
}