#include "utils/utils.h"

// if t is 0, the result has no impact 
double sinc(double t){
    if(t==0){
        return 1;
    }
    return abs(t)<0.002 ? 1-(t*t)/6*(1-(t*t)/20) : sin(t)/t;
}

double mod2pi(double angle)
{
    double out = angle;
    while(out < 0) out+= 2*M_PI;
    while(out >= 2*M_PI) out-= 2*M_PI;
    return out;
}

double modpi(double angle)
{
    double out = angle;
    while(out < -M_PI) out+= 2*M_PI;
    while(out >= M_PI) out-= 2*M_PI;
    return out;
}


int Seed::seed = 0;

void Seed::init_seed(int t_seed)
{
    if(t_seed == 0)
    {
        srand(time(NULL));
        seed = rand();
    }
    else
        seed = t_seed;
}

int Seed::get_seed()
{
    if(seed == 0)
        init_seed(0);
    return seed;
}
