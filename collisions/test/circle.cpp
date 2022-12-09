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