#include "robot.h"
#include <stdio.h>
extern double dt;

Robot::Robot(double x, double y, double theta, double noise)
{
    this->x0=x;
    this->y0=y;
    this->theta0=theta;
    this->noise = noise;
    speed0=0;
    cleanAll();
}

void Robot::cleanAll()
{
    speed_v.clear();
    theta_v.clear();
    x_v.clear();
    y_v.clear();
    x_v.push_back(x0);
    y_v.push_back(y0);
    theta_v.push_back(theta0);
    speed_v.push_back(speed0);
}



void Robot::Clock(double u1, double u2)
{
    double x = x_v.back();
    double y = y_v.back();
    double theta = theta_v.back();
    double vit = speed_v.back();

    double r_noise =0; //-0.5*0.8*noise + 2*0.5*0.8*noise*( (double) rand()/RAND_MAX);
    x=x+dt*vit*cos(theta) + r_noise;
    y=y+dt*vit*sin(theta) + r_noise;
    theta=theta+dt*u1;
    vit=vit+dt*u2;
    speed_v.push_back(vit);
    theta_v.push_back(theta);
    x_v.push_back(x);
    y_v.push_back(y);
}

// Generate a 8 as trajectori
int Robot::generate8(double R, int nb_steps){
    cleanAll();
    int nTour = 1;
    //double n = (int) ((2*M_PI*R) / (V0*dt));
    double V0 = (nTour*4*M_PI*R) / (nb_steps*dt);
    speed_v[0] = V0;
    int sign = 1;
    for(uint i = 0; i < nb_steps ; i++){
        double u1 = sign*(V0/R);
        if(i%(nb_steps/(2*nTour)) == 0) sign*=-1;
        Clock(u1,0);
    }
    return 0;

}
