#include "gtest/gtest.h"
#include "collisions/collisions.hpp"

// intersect segment

TEST(Polygon, IntersectSegment1){
    Polygon p({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Segment s(Point2D(-3, -1), Point2D(1, 4));
    EXPECT_TRUE(intersect(p, s));
}

TEST(Polygon, IntersectSegment2){
    Polygon p({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Segment s(Point2D(-3, -1), Point2D(3, -1));
    EXPECT_FALSE(intersect(p, s));
}

TEST(Polygon, IntersectSegment3){
    Polygon p({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Segment s(Point2D(0, 4), Point2D(4, 0));
    EXPECT_TRUE(intersect(p, s));
}

