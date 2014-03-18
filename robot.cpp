#include "robot.h"
#include <stdio.h>
extern double dt;

Robot::Robot(double x, double y, double theta)
{
    this->x=x;
    this->y=y;
    this->theta=theta;
    vit=0;
}

void Robot::cleanAll()
{
    speed_v.clear();
    theta_v.clear();
    x_v.clear();
    y_v.clear();
    distance_v.clear();
}

double Robot::getDistanceTo(double x0, double y0)
{
    return hypot(this->x - x0, this->y - y0);
}

void Robot::Clock(double u1, double u2)
{   x=x+dt*vit*cos(theta);
    y=y+dt*vit*sin(theta);
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
    //double n = (int) ((2*M_PI*R) / (V0*dt));
    double V0 = (4*M_PI*R) / (nb_steps*dt);
    vit = V0;
    for(uint i = 0; i < nb_steps ; i++){
        double u1 = (i < 0.5*nb_steps) ? (V0/R): -(V0/R);
        Clock(u1,0);
    }
    return 0;

}
