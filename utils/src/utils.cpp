#include "utils/utils.h"

double sinc(double t){
    return abs(t)<0.002 ? 1-(t*t)/6*(1-(t*t)/20) : sin(t)/t;
}

double mod2pi(double angle)
{
    double out = angle;
    while(out < 0) out+= 2*M_PI;
    while(out >= 2*M_PI) out-= 2*M_PI;
    return out;
}
