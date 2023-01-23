#include "dubins/dubins.hpp"

using namespace std;

// returns a vector containing the approximation of a dubin curve
// using n points. If a straight arc is found inside the curve,
// its points will compose the 10% of the total points.
// Points that compose a curve arc will be proportional to
// the length of the arc inside the curve 
std::vector<DubinPoint> DubinCurve::to_points(int n){
    std::vector<DubinPoint> points;
    // s_coeff is for straight line
    double const s_coeff = 0.1;
    double curves_length = get_length();
    for(auto arc:arcs){
        // search for a straight arc (there can be max 1 per curve).
        // if so, reduce the available points for the curves by an amount
        // equal to the number of points dedicated to the straight arc
        if(arc.k == 0){
            n -= (s_coeff * n);
            curves_length -= arc.length;
        }
    }
    for(auto arc:arcs){
        double coeff = arc.k == 0 ? s_coeff : (arc.length / curves_length);
        int arc_points_n = round(coeff * n);
        auto arc_points = arc.to_points(arc_points_n);
        // remove first element from every set of points to avoid duplicates
        // arc_points.erase(arc_points.begin());
        points.insert(points.end(), arc_points.begin(), arc_points.end());
    }

    return points;
}


std::vector<DubinPoint> DubinCurve::to_points_homogeneous(double sens){
    std::vector<DubinPoint> points;
    if(arcs.empty()){
        return points;
    }
    points.push_back(arcs[0].source);
    for(auto arc:arcs){
        auto arc_points = arc.to_points_homogeneous(sens);
        if(!arc_points.empty()){
            points.insert(points.end(), arc_points.begin() + 1, arc_points.end());
        }
    }
    return points;
}

std::vector<DubinPoint> DubinArc::to_points_homogeneous(double sens){
    int const n_points = round(this->length / sens) + 2;
    return to_points(n_points);
}

// returns a vector of n points that approximates the arc
std::vector<DubinPoint> DubinArc::to_points(int n){
    std::vector<DubinPoint> points;
    if(n<2){
        return points;
    }
    // compute the unit segment
    double unit_seg_length = this->length / (n-1);
    for(int i=0;i<n;i++){
        // compute the length of portion of the arc
        double arc_length = unit_seg_length*i;
        // create a smaller dubin arc
        DubinArc new_arc(this->source, arc_length, this->k);
        // get the final point of the smaller dubin arc
        DubinPoint p = new_arc.get_dest();
        points.push_back(p);
    }
    return points;
}

DubinPoint DubinArc::get_dest(){
    double x = this->source.x + this->length * sinc(this->k * this->length / 2.0) * cos(this->source.th + this->k * this->length / 2);
    double y = this->source.y + this->length * sinc(this->k * this->length / 2.0) * sin(this->source.th + this->k * this->length / 2);
    double th = mod2pi(this->source.th + this->k * this->length);
    return DubinPoint(x,y,th);
}

Eigen::Vector4d scale_to_standard(DubinPoint p_start, DubinPoint p_end, const double curvature){    
    double dx = p_end.x - p_start.x;
    double dy = p_end.y - p_start.y;
    double phi = atan2(dy,dx);
    double lambda = hypot(dx,dy)/2;

    double scaled_th0 = mod2pi(p_start.th - phi);     
    double scaled_thf = mod2pi(p_end.th - phi);
    double scaled_kmax = curvature * lambda;
    return Eigen::Vector4d(scaled_th0,scaled_thf,scaled_kmax,lambda);
}

Eigen::Vector3d scale_from_standard(double lambda, double sc_s1, double sc_s2, double sc_s3){
    return Eigen::Vector3d(sc_s1*lambda, sc_s2*lambda, sc_s3*lambda);
}

primitive lsl(double sc_th0, double sc_thf,double sc_kmax){
    double const invk = 1/sc_kmax;
    primitive ret;

    double c=cos(sc_thf)-cos(sc_th0);
    double s=2*sc_kmax+sin(sc_th0)-sin(sc_thf);
    double temp1=atan2(c,s);
    ret.curve1_len=invk*mod2pi(temp1-sc_th0);
    double temp2=2+4*(sc_kmax*sc_kmax)-2*cos(sc_th0-sc_thf)+4*sc_kmax*(sin(sc_th0)-sin(sc_thf));
    if(temp2<0){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.ok=true;
        ret.curve2_len=invk*sqrt(temp2);
        ret.curve3_len=invk*mod2pi(sc_thf-temp1);
    }
    return ret;
}

primitive rsr(double sc_th0, double sc_thf,double sc_kmax){
    double const invk= 1/sc_kmax;
    primitive ret;

    double c = cos(sc_th0) - cos(sc_thf);
    double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(c, s);
    ret.curve1_len=invk*mod2pi(sc_th0-temp1);
    double temp2 = 2 + 4 * (sc_kmax*sc_kmax) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2<0){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.curve2_len = invk * sqrt(temp2);
        ret.curve3_len = invk * mod2pi(temp1 - sc_thf);
        ret.ok = true;
    }
    return ret;
}

primitive lsr(double sc_th0, double sc_thf,double sc_kmax){
    double const invk = 1/sc_kmax;
    primitive ret;

    double c = cos(sc_th0) + cos(sc_thf);
    double s = 2 * sc_kmax + sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(-c, s);
    double temp3 = 4 * (sc_kmax*sc_kmax) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
    if(temp3<0){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.curve2_len=invk*sqrt(temp3);
        double temp2=-atan2(-2,ret.curve2_len*sc_kmax);
        ret.curve1_len=invk*mod2pi(temp1+temp2-sc_th0);
        ret.curve3_len=invk*mod2pi(temp1+temp2-sc_thf);
        ret.ok=true;
    }
    return ret;
}

primitive rsl(double sc_th0, double sc_thf,double sc_kmax){
    double const invk = 1/sc_kmax;
    primitive ret;

    double c = cos(sc_th0) + cos(sc_thf);
    double s = 2 * sc_kmax - sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(c, s);
    double temp3 = 4 * (sc_kmax*sc_kmax) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
    if(temp3<0){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.curve2_len=invk*sqrt(temp3);
        double temp2=atan2(2,ret.curve2_len*sc_kmax);
        ret.curve1_len=invk*mod2pi(sc_th0-temp1+temp2);
        ret.curve3_len=invk*mod2pi(sc_thf-temp1+temp2);
        ret.ok=true;
    }
    return ret;
}

primitive rlr(double sc_th0, double sc_thf,double sc_kmax){
    double const invk = 1/sc_kmax;
    primitive ret;

    double c = cos(sc_th0) - cos(sc_thf);
    double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(c, s);
    double temp2 = 0.125 * (6 - 4 * (sc_kmax*sc_kmax) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
    if(abs(temp2)>1){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.curve2_len=invk*mod2pi(2*M_PI-acos(temp2));
        ret.curve1_len=invk*mod2pi(sc_th0-temp1+0.5*ret.curve2_len*sc_kmax);
        ret.curve3_len=invk*mod2pi(sc_th0-sc_thf+sc_kmax*(ret.curve2_len-ret.curve1_len));
        ret.ok=true;
    }
    return ret;
}

primitive lrl(double sc_th0, double sc_thf,double sc_kmax){
    double const invk = 1/sc_kmax;
    primitive ret;

    double c = cos(sc_thf) - cos(sc_th0);
    double s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(c, s);
    double temp2 = 0.125 * (6 - 4 * (sc_kmax*sc_kmax) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
    if(abs(temp2)>1){
        ret.ok=false;
        ret.curve1_len=0;
        ret.curve2_len=0;
        ret.curve3_len=0;
    }else{
        ret.curve2_len=invk*mod2pi(2*M_PI-acos(temp2));
        ret.curve1_len=invk*mod2pi(temp1-sc_th0+0.5*ret.curve2_len*sc_kmax);
        ret.curve3_len=invk*mod2pi(sc_thf-sc_th0+sc_kmax*(ret.curve2_len-ret.curve1_len));
        ret.ok=true;
    }
    return ret;
}

primitive calculate_primitive(CurveType ct, double sc_th0, double sc_thf,double sc_kmax){
    switch(ct){
        case CurveType::LSL:
            return lsl(sc_th0, sc_thf, sc_kmax);
        case CurveType::RSR:
            return rsr(sc_th0, sc_thf, sc_kmax);
        case CurveType::LSR:
            return lsr(sc_th0, sc_thf, sc_kmax);
        case CurveType::RSL:
            return rsl(sc_th0, sc_thf, sc_kmax);
        case CurveType::RLR:
            return rlr(sc_th0, sc_thf, sc_kmax);
        case CurveType::LRL:
            return lrl(sc_th0, sc_thf, sc_kmax);
        default:
            primitive p;
            p.ok = false;
            return p;
    }
}

// sort curves by increasing order of length using bubble sort
void sort_curves_by_length(std::vector<DubinCurve>& curves){
    if((int)curves.size() < 2){
        return;
    }
    for(int i = 0; i < (int)curves.size(); i++) {
        for(int j = i + 1; j < (int)curves.size(); j++){
            double curr_length = curves.at(i).get_length();
            double next_length = curves.at(j).get_length();
            if(next_length < curr_length) {
                std::swap(curves[i], curves[j]);
            }
        }
   }
}

// return an ordered vector containing every dubin curve that links p_start to p_end
std::vector<DubinCurve> dubin_curves(DubinPoint p_start, DubinPoint p_end, const double curvature){
    std::vector<DubinCurve> curves;
    double const optimal_curves_n = 6;

    const std::map<CurveType, std::vector<ArcType>> CURVE_TO_ARC_TYPES = {
        { CurveType::LSL, {ArcType::Left,  ArcType::Straight,  ArcType::Left} },
        { CurveType::RSR, {ArcType::Right,  ArcType::Straight,  ArcType::Right} },
        { CurveType::LSR, {ArcType::Left,  ArcType::Straight,  ArcType::Right} },
        { CurveType::RSL, {ArcType::Right,  ArcType::Straight,  ArcType::Left} },
        { CurveType::RLR, {ArcType::Right,  ArcType::Left,  ArcType::Right} },
        { CurveType::LRL, {ArcType::Left,  ArcType::Right,  ArcType::Left} }
    };

    Eigen::Vector4d scaled = scale_to_standard(p_start, p_end, curvature);

    for(int i = 0 ; i < optimal_curves_n ; i++){
        CurveType ct = static_cast<CurveType>(i);
        primitive prim = calculate_primitive(ct, scaled(0),scaled(1),scaled(2));
        if(prim.ok){
            Eigen::Vector3d curves_len = scale_from_standard(scaled(3), prim.curve1_len, prim.curve2_len, prim.curve3_len);
            auto arc_types = CURVE_TO_ARC_TYPES.find(ct);
            DubinArc arc0(p_start, curves_len(0), static_cast<int>(arc_types->second[0])*curvature);
            DubinArc arc1(arc0.get_dest(), curves_len(1), static_cast<int>(arc_types->second[1])*curvature);
            DubinArc arc2(arc1.get_dest(), curves_len(2), static_cast<int>(arc_types->second[2])*curvature);
                        
            curves.push_back(DubinCurve(arc0, arc1, arc2));
        }
    }

    sort_curves_by_length(curves);

    return curves;
}

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end, const double curvature){
    auto curves = dubin_curves(p_start, p_end, curvature);
    if(curves.empty()){
        return DubinCurve();
    }
    return curves.at(0);
}

bool intersect(DubinArc arc, Segment2D s){
    // arc is straight, so just compute a single segment and check the intersection
    if(arc.k==0){
        Point2D source(arc.source.x, arc.source.y);
        auto arc_dest = arc.get_dest();
        Point2D dest(arc_dest.x, arc_dest.y);
        Segment2D arc_s(source, dest);
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
        // p_start and p_end are the point boundaries for the check of the intersection
        auto p_start = p1;
        auto p_end = p3;
        // if k < 0 the curve is clockwise, so we need to switch the boundaries
        // (on a cartesian circle we check the boundaries anti-clockwise)
        if(arc.k < 0){
            p_start = p3;
            p_end = p1;
        }
        if(intersect(circle, s, p_start, p_end)){
            return true;
        }
    }
    return false;
}


// check the collision between a dubin curve (3 arcs) and a segment
bool intersect(DubinCurve c, Segment2D s){
    for (auto arc:c.arcs){
        if(intersect(arc, s)){
            return true;
        }
    }
    return false;
}

bool intersect(DubinCurve c, Polygon p){
    Point2D src(c.arcs[0].source.x, c.arcs[0].source.y);
    Point2D mid(c.arcs[1].source.x, c.arcs[1].source.y);
    Point2D dst(c.arcs[2].source.x, c.arcs[2].source.y);
    if(p.contains(src) || p.contains(mid) || p.contains(dst)){
        return true;
    }
    return intersect_sides(c, p);
}

bool intersect_sides(DubinCurve c, Polygon p){
    for(auto side: p.get_sides()){
        if(intersect(c, side)){
            return true;
        }
    }
    return false;
}


// check the intersection between a dubin curve and a polygon,
// but the curve is approximated by a sequence of n points
bool intersect(DubinCurve c, Polygon p, int n){
    std::vector<Segment2D> segments;
    auto points = c.to_points(n);

    // link points with segments
    for(int i=1;i<(int)points.size();i++){
        auto node1 = points.at(i-1);
        auto node2 = points.at(i);
        Point2D new_node1(node1.x, node1.y);
        Point2D new_node2(node2.x, node2.y);
        segments.push_back(Segment2D(new_node1, new_node2));
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

string operator + (string s, DubinPoint& dp)
{
    return s+"("+to_string(dp.x)+";"+to_string(dp.y)+";"+to_string((int)(dp.th*180/M_PI))+")";
}

string operator + (string s, DubinArc& da)
{
    return s+(da.k==-1?"R":(da.k==1?"L":"S"));
}

string operator + (string s, DubinCurve& dc)
{
    return s+dc.arcs.at(0)+dc.arcs.at(1)+dc.arcs.at(2);
}

std::string DubinLink::to_json(double precision){
    std::vector<DubinPoint> points = get_curve().to_points_homogeneous(precision);
    std::string json;

    for(auto &p:points){
        json += "{";
        json += "\"x\":" + to_string(p.x) + ",";
        json += "\"y\":" + to_string(p.y) + ",";
        // highlight first and last points of the curve
        json += (p == points.front() ? "\"marked\": true" : "\"marked\": false");
        json += (p == points.back() ? "}" : "},");
    }
 
    return json;
}