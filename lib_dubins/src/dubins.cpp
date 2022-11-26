#include "lib_dubins/dubins.h"

using namespace std;

std::vector<Point> arc_to_points(DubinArc arc, int n_points, double velocity, bool acc, bool dec){
    std::vector<Point> points;
    // compute the unit segment
    double unit_seg_length = arc.length / n_points;
    for(int i=0;i<n_points;i++){
        // compute the length of portion of the arc
        double arc_length = unit_seg_length*i;
        // create a smaller dubin arc
        DubinArc new_arc(arc.source, arc_length, arc.k);
        // get the final point of the smaller dubin arc
        DubinPoint p = new_arc.get_dest();

        double acceleration = PRECISION_TRAJ * ACCELERATION;
        double deceleration = PRECISION_TRAJ * DECELERATION;

        double v = velocity;
        if(acc && i <= acceleration)
            v *= (i / acceleration);
        else if(dec && n_points - i <= deceleration)
            v *= ((n_points - i) / deceleration);

        double w = v * arc.k;

        points.push_back(Point(p.x, p.y, p.th, v, w));
    }
    return points;
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

// calculate every optimal curve type and returns the best, with its index as identifier  
tuple<primitive, int> calculate_best_primitive(double th0, double thf, double kmax){
    double best_length = DOUBLE_MAX;
    int best_i = -1;
    double const optimal_curves_n = 6;
    primitive best_prim;
    // iterate through optimal curves
    for(int i = 0 ; i < optimal_curves_n ; i++){
        CurveType ct = static_cast<CurveType>(i);
        primitive prim = calculate_primitive(ct,th0, thf, kmax); 
        double length = prim.sc_s1_c + prim.sc_s2_c + prim.sc_s3_c;

        if(prim.ok && length < best_length){
            best_length = length;
            best_prim = prim;
            best_i = i;
        }
    }

    return make_tuple(best_prim, best_i);
}

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end){ 
    
    Eigen::Vector4d scaled = scale_to_standard(p_start, p_end);    //scale the problem to standard values

    Eigen::MatrixXd ksigns (6,3);
    ksigns<<1,  0,  1,  //LSL
            -1, 0,  -1, //RSR
            1,  0,  -1, //LSR
            -1, 0,  1,  //RSL
            -1, 1,  -1, //RLR
            1,  -1, 1;  //LRL

    tuple<primitive, int> best_primitive_result = calculate_best_primitive(scaled(0),scaled(1),scaled(2));

    primitive best_primitive = get<0>(best_primitive_result);
    int pidx = get<1>(best_primitive_result);

    if(pidx>-1){

        // get the lengths of the 3 arcs
        Eigen::Vector3d s = scale_from_standard(scaled(3), best_primitive.sc_s1_c, best_primitive.sc_s2_c, best_primitive.sc_s3_c);  //scale from the standard solution to the general one
        
        DubinArc arc1(p_start, s(0), ksigns(pidx,0)*CURV_MAX);

        DubinArc arc2(arc1.get_dest(), s(1), ksigns(pidx,1)*CURV_MAX);

        DubinArc arc3(arc2.get_dest(), s(2), ksigns(pidx,2)*CURV_MAX);

        DubinCurve final_curve(arc1, arc2, arc3);

        #if DEBUG_MOBILE
            cout << endl << endl << "*******************" << endl;
            cout << ksigns(pidx, 0) << " " << final_curve.arcs[0].length << " - " << ksigns(pidx, 1) << " " <<  final_curve.arcs[1].length << " - " << ksigns(pidx, 2) << " " <<  final_curve.arcs[2].length << endl;
        #endif

        return final_curve;
    }else{
        #if DEBUG_MOBILE
            cout<<"No curve found!"<<endl;
        #endif
        // return default dubin curve
        return DubinCurve();
    }
}