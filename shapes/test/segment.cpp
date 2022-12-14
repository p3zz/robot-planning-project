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