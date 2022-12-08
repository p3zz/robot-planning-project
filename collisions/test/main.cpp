#include "gtest/gtest.h"
#include "collisions/collisions.hpp"

// incident
TEST(SegmentSegment, Incident){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(0, 2));
    EXPECT_TRUE(s1.intersect(s2));
}

// not incident
TEST(SegmentSegment, NotIncident){
    Segment s1(Point2D(0, 0), Point2D(2, 10));
    Segment s2(Point2D(1, 0), Point2D(3, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel
TEST(SegmentSegment, Parallel){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(3, 0), Point2D(5, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// parallel 2
TEST(SegmentSegment, Parallel2){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(2, 0), Point2D(4, 2));
    EXPECT_FALSE(s1.intersect(s2));
}

// overlap
TEST(SegmentSegment, Overlap){
    Segment s1(Point2D(0, 0), Point2D(2, 2));
    Segment s2(Point2D(1, 1), Point2D(3, 3));
    EXPECT_TRUE(s1.intersect(s2));
}

int main(){
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}
