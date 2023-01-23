#include "gtest/gtest.h"
#include "shapes/shapes.hpp"

// intersect

// incident
TEST(Segment2D, IntersectIncident){
    Segment2D s1(Point2D(0, 0), Point2D(2, 2));
    Segment2D s2(Point2D(2, 0), Point2D(0, 2));
    EXPECT_TRUE(intersect(s1, s2));
}

// not incident
TEST(Segment2D, IntersectNotIncident){
    Segment2D s1(Point2D(0, 0), Point2D(2, 10));
    Segment2D s2(Point2D(1, 0), Point2D(3, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// parallel
TEST(Segment2D, IntersectParallel){
    Segment2D s1(Point2D(0, 0), Point2D(2, 2));
    Segment2D s2(Point2D(3, 0), Point2D(5, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// parallel 2
TEST(Segment2D, IntersectParallel2){
    Segment2D s1(Point2D(0, 0), Point2D(2, 2));
    Segment2D s2(Point2D(2, 0), Point2D(4, 2));
    EXPECT_FALSE(intersect(s1, s2));
}

// overlap
TEST(Segment2D, IntersectOverlap){
    Segment2D s1(Point2D(0, 0), Point2D(2, 2));
    Segment2D s2(Point2D(1, 1), Point2D(3, 3));
    EXPECT_TRUE(intersect(s1, s2));
}

// contains
TEST(Segment2D, Contains){
    Segment2D s(Point2D(0, 0), Point2D(2, 2));
    Point2D p(1,1);
    EXPECT_TRUE(s.contains(p));
}

TEST(Segment2D, ContainsEnd){
    Segment2D s(Point2D(0, 0), Point2D(2, 2));
    Point2D p(2,2);
    EXPECT_TRUE(s.contains(p));
}

TEST(Segment2D, ContainsOutsideSameLine){
    Segment2D s(Point2D(2, 3), Point2D(3, 4));
    Point2D p(4,5);
    EXPECT_FALSE(s.contains(p));
}

TEST(Segment2D, ContainsOutsideDifferentLine){
    Segment2D s(Point2D(2, 3), Point2D(3, 4));
    Point2D p(2,4);
    EXPECT_FALSE(s.contains(p));
}

// get angle

TEST(Segment2D, GetAngle1){
    Segment2D s(Point2D(0, 0), Point2D(1, 0));
    EXPECT_EQ(mod2pi(s.get_angle()), 0);
}

TEST(Segment2D, GetAngle2){
    Segment2D s(Point2D(0, 0), Point2D(-1, 0));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI);
}

TEST(Segment2D, GetAngle3){
    Segment2D s(Point2D(0, 0), Point2D(0, 1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 0.5);
}

TEST(Segment2D, GetAngle4){
    Segment2D s(Point2D(0, 0), Point2D(0, -1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 1.5);
}

TEST(Segment2D, GetAngle5){
    Segment2D s(Point2D(0, 0), Point2D(1, 1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 0.25);
}

TEST(Segment2D, GetAngle6){
    Segment2D s(Point2D(0, 0), Point2D(-1, 1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 0.75);
}

TEST(Segment2D, GetAngle7){
    Segment2D s(Point2D(0, 0), Point2D(-1, -1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 1.25);
}

TEST(Segment2D, GetAngle8){
    Segment2D s(Point2D(0, 0), Point2D(1, -1));
    EXPECT_EQ(mod2pi(s.get_angle()), M_PI * 1.75);
}