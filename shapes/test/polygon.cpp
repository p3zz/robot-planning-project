#include "gtest/gtest.h"
#include "shapes/shapes.hpp"

// intersect segment

Polygon polygon({
    Point2D(1, 3),
    Point2D(2, 2),
    Point2D(2, 1),
    Point2D(-1, 1),
    Point2D(-2, 2),
    Point2D(-1, 4)
});

TEST(Polygon, IntersectSegment1){
    Segment s(Point2D(-3, -1), Point2D(1, 4));
    EXPECT_TRUE(intersect(polygon, s));
}

TEST(Polygon, IntersectSegment2){
    Segment s(Point2D(-3, -1), Point2D(3, -1));
    EXPECT_FALSE(intersect(polygon, s));
}

TEST(Polygon, IntersectSegment3){
    Segment s(Point2D(0, 4), Point2D(4, 0));
    EXPECT_TRUE(intersect(polygon, s));
}

// contains

TEST(Polygon, ContainsPointInside1){
    Point2D p(Point2D(1, 2));
    EXPECT_TRUE(polygon.contains(p));
}

TEST(Polygon, ContainsPointOutside1){
    Point2D p(Point2D(3, 1));
    EXPECT_FALSE(polygon.contains(p));
}

TEST(Polygon, ContainsPointOutside2){
    Point2D p(Point2D(-2, 5));
    EXPECT_FALSE(polygon.contains(p));
}

TEST(Polygon, ContainsPointBelongsTo){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(1, 1));
    EXPECT_TRUE(polygon.contains(p));
}

TEST(Polygon, ContainsPointVertex){
    Polygon pol({Point2D(1, 3), Point2D(2, 2), Point2D(2, 1), Point2D(-1, 1), Point2D(-2, 2), Point2D(-1, 4)});
    Point2D p(Point2D(1, 3));
    EXPECT_TRUE(pol.contains(p));
}

// inflate

TEST(Polygon, Inflate1){
    auto new_polygon = inflate(polygon, 2);
    EXPECT_EQ(new_polygon.get_size(), polygon.get_size() * 2);

    auto point = new_polygon.get_v(0);
    EXPECT_NEAR(point.x, 2.42, 0.01);
    EXPECT_NEAR(point.y, 4.42, 0.01);

    point = new_polygon.get_v(1);
    EXPECT_NEAR(point.x, 3.42, 0.01);
    EXPECT_NEAR(point.y, 3.42, 0.01);

    point = new_polygon.get_v(4);
    EXPECT_NEAR(point.x, 2, 0.01);
    EXPECT_NEAR(point.y, -1, 0.01);

    point = new_polygon.get_v(7);
    EXPECT_NEAR(point.x, -3.41, 0.01);
    EXPECT_NEAR(point.y, 0.59, 0.01);
}