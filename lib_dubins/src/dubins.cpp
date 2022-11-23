#include "lib_dubins/dubins.h"

using namespace std;

struct Point traj[int(MAX_LENGTH_TRAJ * PRECISION_TRAJ)];
int len_traj;

// arc (portion of dubin curve)
class DubinArc {
    public: 
        DubinPoint source;
        double length;
        double k;

        DubinArc(DubinPoint source, double length, double k):source{source},length{length},k{k}{}

        DubinPoint get_dest(){
            double x = this->source.x + this->length * sinc(this->k * this->length / 2.0) * cos(this->source.th + this->k * this->length / 2);
            double y = this->source.y + this->length * sinc(this->k * this->length / 2.0) * sin(this->source.th + this->k * this->length / 2);
            double th = mod2pi(this->source.th + this->k * this->length);
            return DubinPoint(x,y,th);
        }
};

class DubinCurve {
    public:
        DubinArc arc1;
        DubinArc arc2;
        DubinArc arc3;

        DubinCurve(DubinArc arc1, DubinArc arc2, DubinArc arc3):arc1{arc1},arc2{arc2},arc3{arc3}{}

        double get_length(){
            return arc1.length + arc2.length + arc3.length; 
        }
};

struct curve{
    double x0, y0, phi0;    //initial position
    double k;               //curvature (+/- kmax for curves, 0 for straights)
    double l;               //length of the curve
    double xf,yf,phif;      //final position
};

struct curves{
    curve c1,c2,c3; //3 segments of the curve
    int pidx;       //type of the final curve (LSL,RSR, ecc.)
    double l;       //total length of the curve
};

struct primitive{ 
    bool ok;
    double sc_s1_c,sc_s2_c,sc_s3_c;
};

Point getTraj(int x)
{
    return traj[x];
}

int getLenTraj()
{
    return len_traj;
}

double sinc(double t){
    double ret;
    if(abs(t)<0.002){
        ret=1-(t*t)/6*(1-(t*t)/20);
    }else{
        ret=sin(t)/t;
    }
    return ret;
}

double mod2pi(double ang)
{
    double out = ang;
    while(out < 0) out+= 2*PI;
    while(out >= 2*PI) out-= 2*PI;
    return out;
}

void reset_trajectory()
{
    len_traj = 0;
}

void draw_curvature(double length, double velocity, double x0, double y0, double th0, double k, bool acc, bool dec)
{
    //compute start and numbers of trajectory points
    int start_traj = len_traj;
    len_traj += length * PRECISION_TRAJ + 1;

    //compute a point (x y th v w) every cm
    for(int i = start_traj; i < len_traj; i++)
    {
        //x y th
        double s = double(i - start_traj)/PRECISION_TRAJ;
        traj[i].x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.0);
        traj[i].y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2.0);
        traj[i].th = mod2pi(th0 + k * s);

        //v with acceleration, deceleration
        if(acc && i - start_traj <= PRECISION_TRAJ * ACCELERATION)
            traj[i].v = velocity * ((i - start_traj) / (PRECISION_TRAJ * ACCELERATION));
        else if(dec && len_traj - i <= PRECISION_TRAJ * DECELERATION)
            traj[i].v = velocity * ((len_traj - i) / (PRECISION_TRAJ * DECELERATION));
        else
            traj[i].v = velocity;

        //w
        traj[i].w = traj[i].v * k;
    }
}

void draw_line(double length, double velocity, double x0, double y0, double th0, bool acc, bool dec)
{
    //compute start and numbers of trajectory points
    int start_traj = len_traj;
    len_traj += length * PRECISION_TRAJ + 1;

    //compute a point (x y th v w) every cm
    for(int i = start_traj; i < len_traj; i++)
    {
        //x y th
        double s = double(i - start_traj)/PRECISION_TRAJ;
        traj[i].x = x0 + s * cos(th0);
        traj[i].y = y0 + s * sin(th0);
        traj[i].th = th0;

        //v with acceleration, deceleration
        if(acc && i - start_traj <= PRECISION_TRAJ * ACCELERATION)
            traj[i].v = velocity * ((i - start_traj) / (PRECISION_TRAJ * ACCELERATION));
        else if(dec && len_traj - i <= PRECISION_TRAJ * DECELERATION)
            traj[i].v = velocity * ((len_traj - i) / (PRECISION_TRAJ * DECELERATION));
        else
            traj[i].v = velocity;

        //w
        traj[i].w = 0;
    }
}

void draw_trajectory(double xi, double yi, double angi, int t1, double len1, int t2, double len2, int t3, double len3)
{
    reset_trajectory();

    //1st part
    if(t1 == S)
    {
        double velocity = VELOCITY_AVG + 0.2;
        if(len1>2.0) velocity = VELOCITY_AVG + 0.5;
        draw_line(len1, velocity, xi, yi, angi, true, false);
    }
    else
        draw_curvature(len1, VELOCITY_AVG, xi, yi, angi, CURV_MAX * double(t1), true, false);

    //2nd part
    if(t2 == S)
    {
        double velocity = VELOCITY_AVG + 0.2;
        if(len2>2.0) velocity = VELOCITY_AVG + 0.5;
        draw_line(len2, velocity, traj[len_traj-1].x, traj[len_traj-1].y, traj[len_traj-1].th, false, false);
    }
    else
        draw_curvature(len2, VELOCITY_AVG, traj[len_traj-1].x,traj[len_traj-1].y,traj[len_traj-1].th, CURV_MAX * double(t2), false, false);

    //3rd part
    if(t3 == S)
    {
        double velocity = VELOCITY_AVG + 0.2;
        if(len3>2.0) velocity = VELOCITY_AVG + 0.5;
        draw_line(len3, velocity, traj[len_traj-1].x,traj[len_traj-1].y,traj[len_traj-1].th, false, true);
    }
    else
        draw_curvature(len3, VELOCITY_AVG, traj[len_traj-1].x,traj[len_traj-1].y,traj[len_traj-1].th, CURV_MAX * double(t3), false, true);
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
        ret.sc_s2_c=invk*mod2pi(2*PI-acos(temp2));
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
        ret.sc_s2_c=invk*mod2pi(2*PI-acos(temp2));
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

bool dubins_shortest_path(DubinPoint p_start, DubinPoint p_end){ 
    
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
        
        draw_trajectory(final_curve.arc1.source.x, final_curve.arc1.source.y, final_curve.arc1.source.th,
                        ksigns(pidx, 0), final_curve.arc1.length, ksigns(pidx, 1), final_curve.arc2.length, ksigns(pidx, 2), final_curve.arc3.length);
        
        #if DEBUG_MOBILE
            cout << endl << endl << "*******************" << endl;
            cout << ksigns(pidx, 0) << " " << final_curve.arc1.length << " - " << ksigns(pidx, 1) << " " <<  final_curve.arc2.length << " - " << ksigns(pidx, 2) << " " <<  final_curve.arc3.length << endl;
        #endif
        return true;
    }else{
        #if DEBUG_MOBILE
            cout<<"No curve found!"<<endl;
        #endif
        return false;
    }
}