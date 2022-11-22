#include "lib_dubins/dubins.h"

using namespace std;

struct Point traj[int(MAX_LENGTH_TRAJ * PRECISION_TRAJ)];
int len_traj;

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

Eigen::Vector4d scale_to_standard(Eigen::Vector3d posin, Eigen::Vector3d posfin){    
    double dx=posfin(0)-posin(0);
    double dy=posfin(1)-posin(1);
    double phi=atan2(dy,dx);
    double lambda=hypot(dx,dy)/2;

    double scaled_th0=mod2pi(posin(2)-phi);     
    double scaled_thf=mod2pi(posfin(2)-phi);
    double scaled_kmax=CURV_MAX*lambda;
    Eigen::Vector4d ret;
    ret<<scaled_th0,scaled_thf,scaled_kmax,lambda;
    return ret;
}

Eigen::Vector3d scale_from_standard(double lambda,double sc_s1,double sc_s2,double sc_s3){
    Eigen::Vector3d ret;
    ret<<sc_s1*lambda,sc_s2*lambda,sc_s3*lambda;
    return ret;
}

primitive calculate_primitive(int index, double sc_th0, double sc_thf,double sc_kmax){
    double invk= 1/sc_kmax;
    double c,s, temp1,temp2,temp3;
    primitive ret;
    switch (index){
    case 0:{    //LSL
        c=cos(sc_thf)-cos(sc_th0);
        s=2*sc_kmax+sin(sc_th0)-sin(sc_thf);
        temp1=atan2(c,s);
        ret.sc_s1_c=invk*mod2pi(temp1-sc_th0);
        temp2=2+4*(sc_kmax*sc_kmax)-2*cos(sc_th0-sc_thf)+4*sc_kmax*(sin(sc_th0)-sin(sc_thf));
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
        break;
    }
    case 1:{    //RSR
        c = cos(sc_th0) - cos(sc_thf);
        s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(c, s);
        ret.sc_s1_c=invk*mod2pi(sc_th0-temp1);
        temp2=2 + 4 * (sc_kmax*sc_kmax) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf));
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
        break;
    }
    case 2:{    //LSR
        c = cos(sc_th0) + cos(sc_thf);
        s = 2 * sc_kmax + sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(-c, s);
        temp3 = 4 * (sc_kmax*sc_kmax) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
        if(temp3<0){
            ret.ok=false;
            ret.sc_s1_c=0;
            ret.sc_s2_c=0;
            ret.sc_s3_c=0;
        }else{
            ret.sc_s2_c=invk*sqrt(temp3);
            temp2=-atan2(-2,ret.sc_s2_c*sc_kmax);
            ret.sc_s1_c=invk*mod2pi(temp1+temp2-sc_th0);
            ret.sc_s3_c=invk*mod2pi(temp1+temp2-sc_thf);
            ret.ok=true;
        }
        break;
    }
    case 3:{    //RSL
        c = cos(sc_th0) + cos(sc_thf);
        s = 2 * sc_kmax - sin(sc_th0) - sin(sc_thf);
        temp1 = atan2(c, s);
        temp3 = 4 * (sc_kmax*sc_kmax) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) + sin(sc_thf));
        if(temp3<0){
            ret.ok=false;
            ret.sc_s1_c=0;
            ret.sc_s2_c=0;
            ret.sc_s3_c=0;
        }else{
            ret.sc_s2_c=invk*sqrt(temp3);
            temp2=atan2(2,ret.sc_s2_c*sc_kmax);
            ret.sc_s1_c=invk*mod2pi(sc_th0-temp1+temp2);
            ret.sc_s3_c=invk*mod2pi(sc_thf-temp1+temp2);
            ret.ok=true;
        }
        break;
    }
    case 4:{    //RLR
        c = cos(sc_th0) - cos(sc_thf);
        s = 2 * sc_kmax - sin(sc_th0) + sin(sc_thf);
        temp1 = atan2(c, s);
        temp2 = 0.125 * (6 - 4 * (sc_kmax*sc_kmax) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
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
        break;
    }
    case 5:{    //LRL
        c = cos(sc_thf) - cos(sc_th0);
        s = 2 * sc_kmax + sin(sc_th0) - sin(sc_thf);
        temp1 = atan2(c, s);
        temp2 = 0.125 * (6 - 4 * (sc_kmax*sc_kmax) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_kmax * (sin(sc_th0) - sin(sc_thf)));
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
        break;
    } 
    default:
        cout<<"An error occurred while calculating the primitive!"<<endl;
        break;
    }
    return ret;
}

curve dubins_arc(double x0, double y0, double phi0, double k, double l){
    curve ret;
    ret.x0=x0;
    ret.y0=y0;
    ret.phi0=phi0;
    ret.k=k;
    ret.l=l;
    ret.xf=ret.x0+l*sinc(k*l/2.0)*cos(phi0+k*l/2);
    ret.yf=y0+l*sinc(k*l/2.0)*sin(phi0+k*l/2);
    ret.phif=mod2pi(phi0+k*l);
    return ret; 
}

curves dubins_curve(Eigen::Vector3d posin, Eigen::Vector3d s, double k0, double k1, double k2){
    curves ret;
    ret.c1=dubins_arc(posin(0),posin(1),posin(2),k0,s(0));
    ret.c2=dubins_arc(ret.c1.xf,ret.c1.yf,ret.c1.phif,k1,s(1));
    ret.c3=dubins_arc(ret.c2.xf,ret.c2.yf,ret.c2.phif,k2,s(2));
    ret.l=ret.c1.l+ret.c2.l+ret.c3.l;
    ret.pidx=-1;
    return ret;
}

bool dubins_shortest_path(double xi, double yi, double angi, double xf, double yf, double angf){ 
    
    Eigen::Vector3d posin;
    Eigen::Vector3d posfin;
    posin  << xi,yi,angi;
    posfin << xf,yf,angf;
    Eigen::Vector4d scaled = scale_to_standard(posin,posfin);    //scale the problem to standard values

    Eigen::MatrixXd ksigns (6,3);
    ksigns<<1,  0,  1,  //LSL
            -1, 0,  -1, //RSR
            1,  0,  -1, //LSR
            -1, 0,  1,  //RSL
            -1, 1,  -1, //RLR
            1,  -1, 1;  //LRL

    int pidx=-1;
    double l=DOUBLE_MAX;
    double lcur=0.0;
    
    double sc_s1,sc_s2,sc_s3;

    for(int i=0;i<6;i++){
        primitive tmp=calculate_primitive(i,scaled(0),scaled(1),scaled(2)); 
        lcur=tmp.sc_s1_c+tmp.sc_s2_c+tmp.sc_s3_c;   //for every given curve type, calculate the length of the relative curve
        if(tmp.ok && lcur<l){
            l=lcur;
            sc_s1=tmp.sc_s1_c;
            sc_s2=tmp.sc_s2_c;
            sc_s3=tmp.sc_s3_c;
            pidx=i; //choose the curve with the shortest total length
        }
    }

    curves finalcurve;
    if(pidx>=0){
        Eigen::Vector3d s= scale_from_standard(scaled(3),sc_s1,sc_s2,sc_s3);  //scale from the standard solution to the general one
        finalcurve=dubins_curve(posin,s,ksigns(pidx,0)*CURV_MAX,ksigns(pidx,1)*CURV_MAX,ksigns(pidx,2)*CURV_MAX);    //calculate all the parameter of the selected curve
        draw_trajectory(xi, yi, angi, ksigns(pidx, 0), finalcurve.c1.l, ksigns(pidx, 1), finalcurve.c2.l, ksigns(pidx, 2), finalcurve.c3.l);
        #if DEBUG_MOBILE
            cout << endl << endl << "*******************" << endl;
            cout << ksigns(pidx, 0) << " " << finalcurve.c1.l << " - " << ksigns(pidx, 1) << " " <<  finalcurve.c2.l << " - " << ksigns(pidx, 2) << " " <<  finalcurve.c3.l << endl;
        #endif
        return true;
    }else{
        #if DEBUG_MOBILE
            cout<<"No curve found!"<<endl;
        #endif
        return false;
    }
}