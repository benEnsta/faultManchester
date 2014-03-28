/**
 * \file      sivia.h
 * \author    Benoit DESROCHERS <benoitdesrochers@ensta-bretagne.org>
 * \version   1.0
 * \date      mar 24, 2014
 * \brief     SIVIA header
 */

#ifndef SIVIA_H
#define SIVIA_H
#include <QObject>

#include "interval/iboolean.h"
#include "interval/interval.h"
#include "interval/box.h"
#include "robot.h"
#include <QDebug>
#include <vector>
#include <list>
#include <fstream>
using namespace std;
typedef vector <vector < interval> > iMatrix;
class SIVIA : public QObject
{
    Q_OBJECT
public:
    explicit SIVIA();

     ~SIVIA();

    // Parametre de l'algorythme
    int N_outliers;

    void runAll2(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);
    void Ctrajectory(box &X, vector<Robot *> *rob);
    vector<int> findOutliers(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);

    // Elementary contractors
    void contractCircle(interval &x0, interval &y0, interval &x1, interval &y1, interval &d);
    void contractCircle(interval& x0,interval& y0, double x1, double y1, interval& d);

    void Incremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double vit, double noise);
    void Decremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double V, double err);


    // tool functions
    box vector2box(vector<box> &T);
    vector<box> box2vector(box X, int boxSize);

};

#endif // SIVIA_H
