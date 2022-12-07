#include "collisions/collisions.hpp"

bool is_bounded(double val, double min, double max){
    return val>=min && val<=max;
}

double Point2D::distance_from(Point2D p){
    return sqrt(pow(x-p.x,2)+pow(y-p.y,2));
}

bool Segment::intersect(Segment s){
    double det = (s.dst.x - s.src.x) * (src.y - dst.y) - (src.x - dst.x) * (s.dst.y - s.src.y);
    // segments are collinear
    if(det == 0){
        // check if the segments overlap
        return this->contains(s.src) || this->contains(s.dst) || s.contains(src) || s.contains(dst);
    }
    double t = ((s.src.y - s.dst.y) * (src.x - s.src.x) + (s.dst.x - s.src.x) * (src.y - s.src.y)) / det;
    double u = ((src.y - dst.y) * (src.x - s.src.x) + (dst.x - src.x) * (src.y - s.src.y)) / det;
    return is_bounded(t, 0, 1) && is_bounded(u, 0, 1);
}

bool Segment::contains(Point2D p){
    return is_bounded(p.x, this->src.x, this->dst.x) && is_bounded(src.y, this->src.y, this->dst.y);
}


Point2D Segment::get_interceptor(double t){
    return Point2D(src.x + t*(dst.x - src.x), src.y + t*(dst.y - src.y));
}

// returns a vector containing the intersections between this circle and a given segment
std::vector<Point2D> Circle::intersect(Segment s){
    std::vector<Point2D> intersections = {};
    double delta_x = s.dst.x - s.src.x;
    double delta_y = s.dst.y - s.src.y;
    double a = delta_x * delta_x + delta_y * delta_y;
    double b = delta_x * (s.src.x - c.x) + delta_y * (s.src.y - c.y);
    double cc = (s.src.x - c.x) * (s.src.x - c.x) + (s.src.y - c.y) * (s.src.y - c.y) - r*r;
    double delta = sqrt(b*b - a*cc);
    double t1 = (-b + delta) / a;
    double t2 = (-b - delta) / a;

    // if t1 or t2 are not bounded between 0 and 1 there are no intersections
    if(!is_bounded(t1, 0, 1) || !is_bounded(t2, 0, 1)){
        return intersections;
    }

    // at least one intersection
    Point2D intersection = s.get_interceptor(t1);
    intersections.push_back(intersection);

    // if t1 and t2 are different, there are two insersections
    if(t1 != t2){
        intersection = s.get_interceptor(t2);
        intersections.push_back(intersection);
    }

    return intersections;
}

double Circle::get_angle(Point2D p){
    return atan2(p.y - c.y, p.x - c.x);
}


