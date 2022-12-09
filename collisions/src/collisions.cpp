#include "collisions/collisions.hpp"

bool is_bounded(double val, double min, double max){
    return val>=min && val<=max;
}

double Point2D::distance_from(Point2D p){
    return sqrt(pow(x-p.x,2)+pow(y-p.y,2));
}

bool Segment::intersect(Segment s){
    double det = (s.dst.x - s.src.x) * (src.y - dst.y) - (src.x - dst.x) * (s.dst.y - s.src.y);
    
    if(det == 0){
        // check if the segments overlap
        return this->contains(s.src) || this->contains(s.dst) || s.contains(src) || s.contains(dst);
    }

    double t = ((s.src.y - s.dst.y) * (src.x - s.src.x) + (s.dst.x - s.src.x) * (src.y - s.src.y)) / det;
    double u = ((src.y - dst.y) * (src.x - s.src.x) + (dst.x - src.x) * (src.y - s.src.y)) / det;
    return is_bounded(t, 0, 1) && is_bounded(u, 0, 1);
}

bool Segment::contains(Point2D p){
    double m = (src.y - dst.y) / (src.x - dst.x);
    double q = (src.x * dst.y - dst.x * src.y) / (src.x - dst.x);
    double const err_threshold = 0.01;
    double err = abs(p.y - (m * p.x) - q);
    bool line_contains_p = is_bounded(err, 0, err_threshold);
    return line_contains_p && is_bounded(p.x, src.x, dst.x) && is_bounded(p.y, src.y, dst.y);
}


Point2D Segment::get_interceptor(double t){
    return Point2D(src.x + t*(dst.x - src.x), src.y + t*(dst.y - src.y));
}

bool Circle::intersect(Segment s, Point2D start, Point2D end){
    std::vector<Point2D> intersections = {};

    if(!this->contains(start) || !this->contains(end)){
        std::cout<<"not containing points"<<std::endl;
        return false;
    }

    double delta_x = s.dst.x - s.src.x;
    double delta_y = s.dst.y - s.src.y;
    double a = pow(delta_x,2) + pow(delta_y,2);
    double b = delta_x * (s.src.x - c.x) + delta_y * (s.src.y - c.y);
    double cc = pow(s.src.x - c.x, 2) + pow(s.src.y - c.y, 2) - pow(r,2);
    double delta = b*b - a*cc;
    if(delta < 0){
        return false;
    }
    delta = sqrt(delta);
    double t1 = (-b + delta) / a;
    double t2 = (-b - delta) / a;
    
    if(is_bounded(t1, 0, 1)){
        intersections.push_back(s.get_interceptor(t1));
    }

    if(is_bounded(t2, 0, 1)){
        intersections.push_back(s.get_interceptor(t2));
    }

    double start_angle = this->get_angle(start);
    double end_angle = this->get_angle(end);

    // TODO optimize the for loop in case that t1 and t2 are bounded and equals 

    bool intersected = false;

    // check if at least one of the intersections stays between the bounds
    for(auto inter: intersections){
        double inter_angle = this->get_angle(inter);

        // https://stackoverflow.com/questions/6270785/how-to-determine-whether-a-point-x-y-is-contained-within-an-arc-section-of-a-c
        if(start_angle < end_angle){
            if(is_bounded(inter_angle, start_angle, end_angle)){
                intersected = true;
            }
        }
        else{
            if(inter_angle < end_angle || inter_angle > start_angle){
                intersected = true;
            }
        }
    }

    return intersected;
}

double Circle::get_angle(Point2D p){
    return atan2(p.y - c.y, p.x - c.x);
}

bool Circle::contains(Point2D p){
    double const err_threshold = 0.01;
    double err = abs(pow(p.x - c.x, 2) + pow(p.y - c.y, 2) - pow(r,2));
    return is_bounded(err, 0, err_threshold);
}

Circle get_circle(Point2D p1, Point2D p2, Point2D p3){
    double x12 = p1.x - p2.x;
    double x13 = p1.x - p3.x;
 
    double y12 = p1.y - p2.y;
    double y13 = p1.y - p3.y;
 
    double y31 = p3.y - p1.y;
    double y21 = p2.y - p1.y;
 
    double x31 = p3.x - p1.x;
    double x21 = p2.x - p1.x;
 
    // x1^2 - x3^2
    double sx13 = pow(p1.x, 2) - pow(p3.x, 2);
 
    // y1^2 - y3^2
    double sy13 = pow(p1.y, 2) - pow(p3.y, 2);
 
    double sx21 = pow(p2.x, 2) - pow(p1.x, 2);
    double sy21 = pow(p2.y, 2) - pow(p1.y, 2);
 
    double f = ((sx13) * (x12)
             + (sy13) * (x12)
             + (sx21) * (x13)
             + (sy21) * (x13))
            / (2 * ((y31) * (x12) - (y21) * (x13)));
    double g = ((sx13) * (y12)
             + (sy13) * (y12)
             + (sx21) * (y13)
             + (sy21) * (y13))
            / (2 * ((x31) * (y12) - (x21) * (y13)));
 
    double c = -pow(p1.x, 2) - pow(p1.y, 2) - 2 * g * p1.x - 2 * f * p1.y;
 
    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    // where centre is (h = -g, k = -f) and radius r
    // as r^2 = h^2 + k^2 - c
    Point2D center(-g, -f);
    double sqr_of_r = center.x * center.x + center.y * center.y - c;
 
    // r is the radius
    float r = sqrt(sqr_of_r);

    return Circle(center, r);

}


