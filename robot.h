#ifndef ROBOT_H
#define ROBOT_H
#include <qmath.h>
#include <vector>

using namespace std;

class Robot
{
public:
    Robot(double x = 0, double y= 0, double theta = 0);
    double x,y,theta,vit;
    vector<double> speed_v;
    vector<double> theta_v;
    vector<double> x_v;
    vector<double> y_v;
    vector<double> distance_v;

    void cleanAll();
    void Clock(double,double);
    double getDistanceTo(double x0, double y0);
    int generate8(double R, int nb_steps=1e10);
};

#endif // ROBOT_H
