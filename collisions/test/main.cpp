#include "gtest/gtest.h"
#include "collisions/collisions.hpp"

// incident
TEST(CollisionsTest, SegmentIntersectSegmentIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(0, 2));
    EXPECT_TRUE(s1.intersect(s2));
}

// not incident
TEST(CollisionsTest, SegmentIntersectSegmenNotIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 10));
    Segment s2(Point2D(1, 0), Point2D(3, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel
TEST(CollisionsTest, SegmentIntersectSegmentParallel){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(3, 0), Point2D(5, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel 2
TEST(CollisionsTest, SegmentIntersectSegmentParallel2){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(4, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// overlap
TEST(CollisionsTest, SegmentIntersectSegmentOverlap){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(1, 1), Point2D(3, 3));
    EXPECT_TRUE(s1.intersect(s2));
}

int main(){
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
