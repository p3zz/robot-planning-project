#include "collisions/collisions.hpp"

bool is_bounded(double val, double min, double max){
    return val>=min && val<=max;
}

bool Segment::intersect(Segment s){
    double det = (s.q.x - s.p.x) * (p.y - q.y) - (p.x - q.x) * (s.q.y - s.p.y);
    // segments are collinear
    if(det == 0){
        // check if the segments overlap
        return this->contains(s.p) || this->contains(s.q) || s.contains(p) || s.contains(q);
    }
    double t = ((s.p.y - s.q.y) * (p.x - s.p.x) + (s.q.x - s.p.x) * (p.y - s.p.y)) / det;
    double u = ((p.y - q.y) * (p.x - s.p.x) + (q.x - p.x) * (p.y - s.p.y)) / det;
    return is_bounded(t, 0, 1) && is_bounded(u, 0, 1);
}

bool Segment::contains(Point p){
    return is_bounded(p.x, this->p.x, this->q.x) && is_bounded(p.y, this->p.y, this->q.y);
}


Point Segment::get_interceptor(double t){
    return Point(p.x + t*(q.x - p.x), p.y + t*(q.y - p.y));
}

// returns a vector containing the intersections between this circle and a given segment
std::vector<Point> Circle::intersect(Segment s){
    std::vector<Point> intersections = {};
    double delta_x = s.q.x - s.p.x;
    double delta_y = s.q.y - s.p.y;
    double a = delta_x * delta_x + delta_y * delta_y;
    double b = delta_x * (s.p.x - center.x) + delta_y * (s.p.y - center.y);
    double c = (s.p.x - center.x) * (s.p.x - center.x) + (s.p.y - center.y) * (s.p.y - center.y) - radius*radius;
    double delta = sqrt(b*b - a*c);
    double t1 = (-b + delta) / a;
    double t2 = (-b - delta) / a;

    // if t1 or t2 are not bounded between 0 and 1 there are no intersections
    if(!is_bounded(t1, 0, 1) || !is_bounded(t2, 0, 1)){
        return intersections;
    }

    // at least one intersection
    Point intersection = s.get_interceptor(t1);
    intersections.push_back(intersection);

    // if t1 and t2 are different, there are two insersections
    if(t1 != t2){
        intersection = s.get_interceptor(t2);
        intersections.push_back(intersection);
    }

    return intersections;
}

double Circle::get_angle(Point p){
    return atan2(p.y - center.y, p.x - center.x);
}


