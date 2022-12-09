#include "gtest/gtest.h"
#include "collisions/collisions.hpp"

// intersect

// incident
TEST(Segment, IntersectIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(0, 2));
    EXPECT_TRUE(s1.intersect(s2));
}

// not incident
TEST(Segment, IntersectNotIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 10));
    Segment s2(Point2D(1, 0), Point2D(3, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel
TEST(Segment, IntersectParallel){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(3, 0), Point2D(5, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel 2
TEST(Segment, IntersectParallel2){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(4, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// overlap
TEST(Segment, IntersectOverlap){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(1, 1), Point2D(3, 3));
    EXPECT_TRUE(s1.intersect(s2));
}
