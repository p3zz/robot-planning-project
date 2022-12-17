#include "gtest/gtest.h"
#include "shapes/shapes.hpp"

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

// contains

TEST(Polygon, ContainsPointInside1){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(1, 2));
    EXPECT_TRUE(pol.contains(p));
}

TEST(Polygon, ContainsPointOutside1){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(3, 1));
    EXPECT_FALSE(pol.contains(p));
}

TEST(Polygon, ContainsPointOutside2){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(-2, 5));
    EXPECT_FALSE(pol.contains(p));
}

TEST(Polygon, ContainsPointBelongsTo){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(1, 1));
    EXPECT_TRUE(pol.contains(p));
}

TEST(Polygon, ContainsPointVertex){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(1, 3));
    EXPECT_TRUE(pol.contains(p));
}