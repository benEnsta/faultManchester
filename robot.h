#ifndef ROBOT_H
#define ROBOT_H
#include <qmath.h>
#include <vector>

using namespace std;

class robot
{
public:
    robot(double x = 0, double y= 0, double theta = 0);
    double x,y,theta,vit;
    void Clock(double,double);
    double getDistanceTo(double x0, double y0);
};

#endif // ROBOT_H
