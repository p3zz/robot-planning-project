#include "dubins/dubins.hpp"

using namespace std;

std::vector<DubinPoint> DubinCurve::get_trajectory(){
    std::vector<DubinPoint> points = {};

    points.push_back(arcs[0].source);

    for(auto arc:arcs){
        int arc_points_n = int((arc.length / get_length()) * PRECISION_TRAJ);
        auto arc_points = arc.to_points(arc_points_n);
        points.insert(points.end(), arc_points.begin(), arc_points.end());
    }

    return points;
}

std::vector<DubinPoint> DubinArc::to_points(int n_points){
    std::vector<DubinPoint> points;
    // compute the unit segment
    double unit_seg_length = this->length / n_points;
    for(int i=1;i<=n_points;i++){
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

Eigen::Vector4d scale_to_standard(DubinPoint p_start, DubinPoint p_end){    
    double dx = p_end.x - p_start.x;
    double dy = p_end.y - p_start.y;
    double phi = atan2(dy,dx);
    double lambda = hypot(dx,dy)/2;

    double scaled_th0 = mod2pi(p_start.th - phi);     
    double scaled_thf = mod2pi(p_end.th - phi);
    double scaled_kmax = CURV_MAX*lambda;
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
    ret.sc_s1_c=invk*mod2pi(temp1-sc_th0);
    double temp2=2+4*(sc_kmax*sc_kmax)-2*cos(sc_th0-sc_thf)+4*sc_kmax*(sin(sc_th0)-sin(sc_thf));
    if(temp2<0){
        ret.ok=false;
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.ok=true;
        ret.sc_s2_c=invk*sqrt(temp2);
        ret.sc_s3_c=invk*mod2pi(sc_thf-temp1);
    }
    return ret;
}

primitive rsr(double sc_th0, double sc_thf,double sc_kmax){
    double const invk= 1/sc_kmax;
    primitive ret;

    double c = cos(sc_th0) - cos(sc_thf);
    double s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(c, s);
    ret.sc_s1_c=invk*mod2pi(sc_th0-temp1);
    double temp2 = 2 + 4 * (sc_kmax*sc_kmax) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
    if(temp2<0){
        ret.ok=false;
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.sc_s2_c = invk * sqrt(temp2);
        ret.sc_s3_c = invk * mod2pi(temp1 - sc_thf);
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
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.sc_s2_c=invk*sqrt(temp3);
        double temp2=-atan2(-2,ret.sc_s2_c*sc_kmax);
        ret.sc_s1_c=invk*mod2pi(temp1+temp2-sc_th0);
        ret.sc_s3_c=invk*mod2pi(temp1+temp2-sc_thf);
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
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.sc_s2_c=invk*sqrt(temp3);
        double temp2=atan2(2,ret.sc_s2_c*sc_kmax);
        ret.sc_s1_c=invk*mod2pi(sc_th0-temp1+temp2);
        ret.sc_s3_c=invk*mod2pi(sc_thf-temp1+temp2);
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
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.sc_s2_c=invk*mod2pi(2*M_PI-acos(temp2));
        ret.sc_s1_c=invk*mod2pi(sc_th0-temp1+0.5*ret.sc_s2_c*sc_kmax);
        ret.sc_s3_c=invk*mod2pi(sc_th0-sc_thf+sc_kmax*(ret.sc_s2_c-ret.sc_s1_c));
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
        ret.sc_s1_c=0;
        ret.sc_s2_c=0;
        ret.sc_s3_c=0;
    }else{
        ret.sc_s2_c=invk*mod2pi(2*M_PI-acos(temp2));
        ret.sc_s1_c=invk*mod2pi(temp1-sc_th0+0.5*ret.sc_s2_c*sc_kmax);
        ret.sc_s3_c=invk*mod2pi(sc_thf-sc_th0+sc_kmax*(ret.sc_s2_c-ret.sc_s1_c));
        ret.ok=true;
    }
    return ret;
}

primitive calculate_primitive(CurveType ct, double sc_th0, double sc_thf,double sc_kmax){
    primitive p;
    switch(ct){
        case CurveType::LSL:
            p = lsl(sc_th0, sc_thf, sc_kmax);
            break;
        case CurveType::RSR:
            p = rsr(sc_th0, sc_thf, sc_kmax);
            break;
        case CurveType::LSR:
            p = lsr(sc_th0, sc_thf, sc_kmax);
            break;
        case CurveType::RSL:
            p = rsl(sc_th0, sc_thf, sc_kmax);
            break;
        case CurveType::RLR:
            p = rlr(sc_th0, sc_thf, sc_kmax);
            break;
        case CurveType::LRL:
            p = lrl(sc_th0, sc_thf, sc_kmax);
            break;
        default:
            cout<<"An error occurred while calculating the primitive!"<<endl;
            break;           
    }
    return p;
}

// calculate every optimal curve type and returns the best
tuple<primitive, CurveType> calculate_best_primitive(double th0, double thf, double kmax){

    double const optimal_curves_n = 6;
    double best_length = DOUBLE_MAX;
    primitive best_prim;
    CurveType best_ct = CurveType::None;

    // iterate through optimal curves
    for(int i = 0 ; i < optimal_curves_n ; i++){
        CurveType ct = static_cast<CurveType>(i);
        primitive prim = calculate_primitive(ct, th0, thf, kmax); 
        double length = prim.sc_s1_c + prim.sc_s2_c + prim.sc_s3_c;

        if(prim.ok && length < best_length){
            best_length = length;
            best_prim = prim;
            best_ct = ct;
        }
    }

    return make_tuple(best_prim, best_ct);
}

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end){ 
    
    Eigen::Vector4d scaled = scale_to_standard(p_start, p_end);    //scale the problem to standard values

    tuple<primitive, CurveType> best_primitive_result = calculate_best_primitive(scaled(0),scaled(1),scaled(2));

    std::map<CurveType, std::vector<ArcType>> curve_to_arc_types = {
        { CurveType::LSL, {ArcType::Left,  ArcType::Straight,  ArcType::Left} },
        { CurveType::RSR, {ArcType::Right,  ArcType::Straight,  ArcType::Right} },
        { CurveType::LSR, {ArcType::Left,  ArcType::Straight,  ArcType::Right} },
        { CurveType::RSL, {ArcType::Right,  ArcType::Straight,  ArcType::Left} },
        { CurveType::RLR, {ArcType::Right,  ArcType::Left,  ArcType::Right} },
        { CurveType::LRL, {ArcType::Left,  ArcType::Right,  ArcType::Left} }
    };

    primitive best_primitive = get<0>(best_primitive_result);
    CurveType best_ct = get<1>(best_primitive_result);

    if(best_ct == CurveType::None){
        #if DEBUG_MOBILE
            cout<<"No curve found!"<<endl;
        #endif
        // return default dubin curve
        return DubinCurve();
    }

    // get the lengths of the 3 arcs
    Eigen::Vector3d s = scale_from_standard(scaled(3), best_primitive.sc_s1_c, best_primitive.sc_s2_c, best_primitive.sc_s3_c);  //scale from the standard solution to the general one

    auto arc_types = curve_to_arc_types.find(best_ct);
    
    DubinArc arc0(p_start, s(0), static_cast<int>(arc_types->second[0])*CURV_MAX);

    DubinArc arc1(arc0.get_dest(), s(1), static_cast<int>(arc_types->second[1])*CURV_MAX);

    DubinArc arc2(arc1.get_dest(), s(2), static_cast<int>(arc_types->second[2])*CURV_MAX);

    DubinCurve final_curve(arc0, arc1, arc2);

    #if DEBUG_MOBILE
        cout << endl << endl << "Curve found" << endl;
    #endif

    return final_curve;

}