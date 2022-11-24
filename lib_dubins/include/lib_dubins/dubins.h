#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <utils/utils.h>


#define DEBUG_MOBILE true

#define S 0
#define L 1
#define R -1

#define MAX_LENGTH_TRAJ 500.0 // 100 meter max
#define PRECISION_TRAJ 100.0  // cms precision
#define DECELERATION 0.1      // 10 cms of deceleration
#define ACCELERATION 0.1      // 10 cms of acceleration
#define CURV_MAX 1.0          // max curvature
#define VELOCITY_AVG 0.5      // main velocity

#define DOUBLE_MAX 1.79769e+308

struct Point //structure of point: set all 5 parameters for every point, then len_traj (length of non-empty part of array), when you draw a trajectory!
{
    double x;
    double y;
    double th;
    double v;
    double w;
};

enum CurveType{
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL
};

// point (start/end of a dubin arc)
class DubinPoint{
    public: 
        double x;
        double y;
        double th;

        DubinPoint():x{0},y{0},th{0}{}
        DubinPoint(double x, double y, double th):x{x},y{y},th{th}{}

};

// arc (portion of dubin curve)
class DubinArc {
    public: 
        DubinPoint source;
        double length;
        double k;

        DubinArc():source{DubinPoint()}, length{0}, k{0}{};
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

        DubinCurve():arc1{DubinArc()},arc2{DubinArc()},arc3{DubinArc()}{}
        DubinCurve(DubinArc arc1, DubinArc arc2, DubinArc arc3):arc1{arc1},arc2{arc2},arc3{arc3}{}

        double get_length(){
            return arc1.length + arc2.length + arc3.length; 
        }
};

struct primitive{ 
    bool ok;
    double sc_s1_c,sc_s2_c,sc_s3_c;
};

class DubinDrawer{

    private:

        DubinCurve curve;
        int len_traj;

        void draw_arc(DubinArc arc, double velocity, bool acc, bool dec){
            //compute start and numbers of trajectory points
            int start_traj = this->len_traj;
            this->len_traj += arc.length * PRECISION_TRAJ + 1;

            int curve_type = int(arc.k / CURV_MAX);

            //compute a point (x y th v w) every cm
            for(int i = start_traj; i < this->len_traj; i++)
            {
                //x y th
                double s = double(i - start_traj)/PRECISION_TRAJ;

                // arc is straight
                if(curve_type == S){
                    this->traj[i].x = arc.source.x + s * cos(arc.source.th);
                    this->traj[i].y = arc.source.y + s * sin(arc.source.th);
                    this->traj[i].th = arc.source.th;
                }

                // arc is curve
                else{
                    this->traj[i].x = arc.source.x + s * sinc(arc.k * s / 2.0) * cos(arc.source.th + arc.k * s / 2.0);
                    this->traj[i].y = arc.source.y + s * sinc(arc.k * s / 2.0) * sin(arc.source.th + arc.k * s / 2.0);
                    this->traj[i].th = mod2pi(arc.source.th + arc.k * s);
                }                

                //v with acceleration, deceleration
                if(acc && i - start_traj <= PRECISION_TRAJ * ACCELERATION)
                    this->traj[i].v = velocity * ((i - start_traj) / (PRECISION_TRAJ * ACCELERATION));
                else if(dec && this->len_traj - i <= PRECISION_TRAJ * DECELERATION)
                    this->traj[i].v = velocity * ((this->len_traj - i) / (PRECISION_TRAJ * DECELERATION));
                else
                    this->traj[i].v = velocity;

                if(curve_type == S){
                    this->traj[i].w = 0;
                }
                else{
                    this->traj[i].w = traj[i].v * arc.k;
                }
            }
        }

    public:

        Point traj[int(MAX_LENGTH_TRAJ * PRECISION_TRAJ)];
    
        DubinDrawer(DubinCurve curve): curve{curve}, len_traj{0}{}

        Point* draw_trajectory(){

            double const length_threshold = 2.0;            
            // reset trajectory
            this->len_traj = 0;

            //1st arc
            double velocity = VELOCITY_AVG;

            // if the arc is straight, go faster
            if(int(this->curve.arc1.k / CURV_MAX) == S){
                velocity = this->curve.arc1.length > length_threshold? VELOCITY_AVG + 0.5 : VELOCITY_AVG + 0.2;
            }
            draw_arc(curve.arc1, velocity, true, false);

            //2nd arc
            velocity = VELOCITY_AVG;

            if(int(this->curve.arc2.k / CURV_MAX) == S){
                velocity = this->curve.arc2.length > length_threshold? VELOCITY_AVG + 0.5 : VELOCITY_AVG + 0.2;
            }
            DubinArc arc2(DubinPoint(this->traj[this->len_traj-1].x, this->traj[this->len_traj-1].y, this->traj[this->len_traj-1].th), this->curve.arc2.length, this->curve.arc2.k); 
            draw_arc(arc2, velocity, true, false);

            //3rd arc
            velocity = VELOCITY_AVG;

            if(int(this->curve.arc3.k / CURV_MAX) == S){
                velocity = this->curve.arc3.length > length_threshold? VELOCITY_AVG + 0.5 : VELOCITY_AVG + 0.2;
            }
            DubinArc arc3(DubinPoint(this->traj[this->len_traj-1].x, this->traj[this->len_traj-1].y, this->traj[this->len_traj-1].th), this->curve.arc3.length, this->curve.arc3.k); 
            draw_arc(arc3, velocity, true, false);

            return traj;
        }

        int get_length(){
            return len_traj;
        }

};

DubinCurve dubins_shortest_path(DubinPoint p_start, DubinPoint p_end);

#endif