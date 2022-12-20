#include "shapes/shapes.hpp"

bool is_bounded(double val, double min, double max){
    return val>=min && val<=max;
}

bool intersect(Segment s1, Segment s2){
    double det = (s2.node2.x - s2.node1.x) * (s1.node1.y - s1.node2.y) - (s1.node1.x - s1.node2.x) * (s2.node2.y - s2.node1.y);
    
    if(det == 0){
        // check if the segments overlap
        return s1.contains(s2.node1) || s1.contains(s2.node2) || s2.contains(s1.node1) || s2.contains(s1.node2);
    }

    double t = ((s2.node1.y - s2.node2.y) * (s1.node1.x - s2.node1.x) + (s2.node2.x - s2.node1.x) * (s1.node1.y - s2.node1.y)) / det;
    double u = ((s1.node1.y - s1.node2.y) * (s1.node1.x - s2.node1.x) + (s1.node2.x - s1.node1.x) * (s1.node1.y - s2.node1.y)) / det;
    return is_bounded(t, 0, 1) && is_bounded(u, 0, 1);
}

bool intersect(Circle circle, Segment s, Point2D start, Point2D end){
    std::vector<Point2D> intersections = {};

    if(!circle.contains(start) || !circle.contains(end)){
        return false;
    }

    double delta_x = s.node2.x - s.node1.x;
    double delta_y = s.node2.y - s.node1.y;
    double a = pow(delta_x,2) + pow(delta_y,2);
    double b = delta_x * (s.node1.x - circle.c.x) + delta_y * (s.node1.y - circle.c.y);
    double cc = pow(s.node1.x - circle.c.x, 2) + pow(s.node1.y - circle.c.y, 2) - pow(circle.r,2);
    double delta = b*b - a*cc;
    if(delta < 0 || a == 0){
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

    double start_angle = circle.get_angle(start);
    double end_angle = circle.get_angle(end);

    // TODO optimize the for loop in case that t1 and t2 are bounded and equals 

    bool intersected = false;

    // check if at least one of the intersections stays between the bounds
    for(auto inter: intersections){
        double inter_angle = circle.get_angle(inter);

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

bool intersect(Polygon p, Segment s){
    auto sides = p.get_sides();
    for(auto side: sides){
        if(intersect(s, side)){
            return true;
        }
    }
    return false;
}

double Point2D::distance_from(Point2D p){
    return sqrt(pow(x-p.x,2)+pow(y-p.y,2));
}

bool Segment::contains(Point2D p){
    double m = (node1.y - node2.y) / (node1.x - node2.x);
    double q = (node1.x * node2.y - node2.x * node1.y) / (node1.x - node2.x);
    double const err_threshold = 0.05;
    double err = abs(p.y - (m * p.x) - q);
    bool line_contains_p = is_bounded(err, 0, err_threshold);
    return line_contains_p && is_bounded(p.x, node1.x, node2.x) && is_bounded(p.y, node1.y, node2.y);
}


Point2D Segment::get_interceptor(double t){
    return Point2D(node1.x + t*(node2.x - node1.x), node1.y + t*(node2.y - node1.y));
}

double Circle::get_angle(Point2D p){
    return atan2(p.y - c.y, p.x - c.x);
}

bool Circle::contains(Point2D p){
    double const err_threshold = 0.05;
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

std::vector<Segment> Polygon::get_sides(){
    std::vector<Segment> sides = {};
    for(int i=1;i<(int)vertexes.size();i++){
        Segment side(vertexes[i-1],vertexes[i]);
        sides.push_back(side);
    }
    Segment side(vertexes.back(),vertexes.front());
    sides.push_back(side);
    return sides;
}

bool Polygon::contains(Point2D p){
    int pos = 0;
    int neg = 0;

    auto sides = this->get_sides();
    for (auto side: sides){
        //Compute the cross product
        double d = (p.x - side.node1.x) * (side.node2.y - side.node1.y) - (p.y - side.node1.y) * (side.node2.x - side.node1.x);

        if (d > 0) pos++;
        if (d < 0) neg++;

        //If the sign changes, then point is outside
        if (pos > 0 && neg > 0)
            return false;
    }

    //If no change in direction, then on same side of all segments, and thus inside
    return true;
}

bool intersect(DubinArc arc, Segment s){
    // arc is straight, so just compute a single segment and check the insersection
    if(arc.k==0){
        Point2D source(arc.source.x, arc.source.y);
        auto arc_dest = arc.get_dest();
        Point2D dest(arc_dest.x, arc_dest.y);
        Segment arc_s(source, dest);
        if(intersect(s, arc_s)){
            return true;
        }
    // arc is curve, so build a circle and check the intersection with the arc
    }else{
        // compute the point that is in the middle of the arc. In order to build a full circle
        // we need 3 points: the source point of the arc, the destination, and a middle point
        Point2D p1(arc.source.x, arc.source.y);        

        DubinArc arc_mid(arc.source, arc.length*0.5, arc.k);
        DubinPoint p_mid = arc_mid.get_dest();
        Point2D p2(p_mid.x, p_mid.y);

        DubinPoint p_dest = arc.get_dest();
        Point2D p3(p_dest.x, p_dest.y);
        // build the circle and check the interection

        Circle circle = get_circle(p1, p2, p3);
        if(intersect(circle, s, p1, p3)){
            return true;
        }
    }
    return false;
}


// check the collision between a dubin curve (3 arcs) and a segment
bool intersect(DubinCurve c, Segment s){
    for (auto arc:c.arcs){
        if(intersect(arc, s)){
            return true;
        }
    }
    return false;
}

bool intersect(DubinCurve c, Polygon p){
    for(auto side: p.get_sides()){
        if(intersect(c, side)){
            return true;
        }
    }
    return false;
}


bool intersect(DubinCurve c, Polygon p, int n){
    std::vector<Segment> segments;
    auto points = c.to_points(n);

    // link points with segments
    for(int i=1;i<(int)points.size();i++){
        auto node1 = points.at(i-1);
        auto node2 = points.at(i);
        Point2D new_node1(node1.x, node1.y);
        Point2D new_node2(node2.x, node2.y);
        segments.push_back(Segment(new_node1, new_node2));
    }

    for(auto dubin_seg: segments){
        for(auto seg: p.get_sides()){
            if(intersect(dubin_seg, seg)){
                return true;
            }
        }
    }
    return false;
}

double distance(Point2D p1, Point2D p2){
    return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

Point2D Point2D::traslate(double offset_x, double offset_y){
    return Point2D(x + offset_x, y + offset_y);
}

Segment Segment::traslate(double offset){
    double slope = get_slope();
    double perpendicular_slope;

    if(std::isinf(slope)){
        perpendicular_slope = 0;
    } else if(slope == 0){
        perpendicular_slope = std::numeric_limits<double>::infinity();
    }else{
        perpendicular_slope = -1 / slope;
    }
    double th = atan(perpendicular_slope);
    double offset_x = offset * cosf(th);
    double offset_y = offset * sinf(th);
    Point2D node1_new = node1.traslate(offset_x, offset_y);
    Point2D node2_new = node2.traslate(offset_x, offset_y);
    return Segment(node1_new, node2_new);
}

double Segment::get_slope(){
    double delta_x = node2.x - node1.x;
    double delta_y = node2.y - node1.y; 
    if(delta_x == 0){
        return std::numeric_limits<double>::infinity();
    }
    return delta_y / delta_x;
}