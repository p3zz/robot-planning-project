#include "gtest/gtest.h"
#include "shapes/shapes.hpp"

TEST(Circle, Contains1){
    Circle c(Point2D(0, 0), 1);
    Point2D p(0.87, 0.5);
}