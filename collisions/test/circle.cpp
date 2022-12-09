#include "gtest/gtest.h"
#include "collisions/collisions.hpp"

// contains

TEST(Circle, Contains1){
    Circle c(Point2D(0, 0), 1);
    Point2D p(0.87, 0.5);
    EXPECT_TRUE(c.contains(p));
}

TEST(Circle, Contains2){
    Circle c(Point2D(0, 0), 1);
    Point2D p(0, -1);
    EXPECT_TRUE(c.contains(p));
}

TEST(Circle, ContainsInside){
    Circle c(Point2D(1, 1), 2);
    Point2D p(2,2);
    EXPECT_FALSE(c.contains(p));
}

TEST(Circle, ContainsOutside){
    Circle c(Point2D(1, 1), 2);
    Point2D p(4,1);
    EXPECT_FALSE(c.contains(p));
}

// get_angle

TEST(Circle, GetAngle1){
    Circle c(Point2D(1, 1), 2);
    Point2D p(2,2.73);
    double angle = c.get_angle(p);
    EXPECT_NEAR(angle, 1.0466, 0.01);
}

TEST(Circle, GetAngle2){
    Circle c(Point2D(0, 0), 2);
    Point2D p(-1.73, 1);
    double angle = c.get_angle(p);
    EXPECT_NEAR(angle, 2.61694, 0.01);
}

// intersect

TEST(Circle, IntersectOnceTangentInBounds){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(-4,2), Point2D(1,2));
    Point2D start_bound(1,1.73);
    Point2D end_bound(-1,1.73);
    EXPECT_TRUE(c.intersect(s, start_bound, end_bound));
}

TEST(Circle, IntersectOnceTangentInBounds2){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(1,-2), Point2D(4,2));
    Point2D start_bound(0.7, -1.87);
    Point2D end_bound(1.93, -0.53);
    EXPECT_TRUE(c.intersect(s, start_bound, end_bound));
}

TEST(Circle, IntersectOnceTangentOutBounds1){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(-4,2), Point2D(1,2));
    Point2D start_bound(0.7, -1.87);
    Point2D end_bound(1.93, -0.53);
    EXPECT_FALSE(c.intersect(s, start_bound, end_bound));
}

TEST(Circle, IntersectOnceTangentOutBounds2){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(1,-2), Point2D(4,2));
    Point2D start_bound(1,1.73);
    Point2D end_bound(-1,1.73);
    EXPECT_FALSE(c.intersect(s, start_bound, end_bound));
}


TEST(Circle, IntersectOnce1){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(0, 1), Point2D(3,-1));
    Point2D start_bound(1.93, -0.53);
    Point2D end_bound(1.96, 0.4);
    EXPECT_TRUE(c.intersect(s, start_bound, end_bound));
}

TEST(Circle, IntersectOnce2){
    Circle c(Point2D(0, 0), 2);
    Segment s(Point2D(-3, 3), Point2D(-1, 1));
    Point2D start_bound(-1.41, 1.41);
    Point2D end_bound(-2, 0.13);
    EXPECT_TRUE(c.intersect(s, start_bound, end_bound));
}

// TEST(Circle, IntersectTwice1){
//     Circle c(Point2D(0, 0), 2);
//     Segment s(Point2D(-3, 3), Point2D(-1,-2));
//     Point2D start_bound(1.96, 0.4);
//     Point2D end_bound(1.93, -0.53);
//     EXPECT_FALSE(c.intersect(s, start_bound, end_bound));
// }

// TEST(Circle, IntersectOutside){
//     Circle c(Point2D(0, 0), 2);
//     Segment s(Point2D(3, 1), Point2D(2, 3);
//     EXPECT_FALSE(c.intersect(s));
// }

// TEST(Circle, IntersectInside){
//     Circle c(Point2D(0, 0), 2);
//     Segment s(Point2D(3, 1), Point2D(2, 3);
//     EXPECT_FALSE(c.intersect(s));
// }