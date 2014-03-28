/**
 * \file      robot.h.
 * \author    Benoit DESROCHERS <benoitdesrochers@ensta-bretagne.org>
 * \version   1.0
 * \date      mar 24, 2014
 * \brief     Defined robot model
 *
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <qmath.h>
#include <vector>

using namespace std;

class Robot
{
public:
    Robot(double x = 0, double y= 0, double theta = 0, double noise = 0.000);
    double x0,y0,theta0,speed0, noise;
    vector<double> speed_v;
    vector<double> theta_v;
    vector<double> x_v;
    vector<double> y_v;

    void cleanAll();
    void Clock(double,double);
    int generate8(double R, int nb_steps_per_tour=100, int nb_steps=120);
};

#endif // ROBOT_H
