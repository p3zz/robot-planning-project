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