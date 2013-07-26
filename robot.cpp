#include "robot.h"
#include <stdio.h>
extern double dt;

robot::robot(double x, double y, double theta)
{
    this->x=x;
    this->y=y;
    this->theta=theta;
    vit=0;
}

void robot::cleanAll()
{
    speed_v.clear();
    theta_v.clear();
    x_v.clear();
    y_v.clear();
    distance_v.clear();
}

double robot::getDistanceTo(double x0, double y0)
{
    return hypot(this->x - x0, this->y - y0);
}

void robot::Clock(double u1, double u2)
{   x=x+dt*vit*cos(theta);
    y=y+dt*vit*sin(theta);
    theta=theta+dt*u1;
    vit=vit+dt*u2;
    speed_v.push_back(vit);
    theta_v.push_back(theta);
    x_v.push_back(x);
    y_v.push_back(y);
}
