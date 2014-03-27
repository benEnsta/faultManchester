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

     typedef void (SIVIA::*AContractorPtr)(box &);
    ~SIVIA();

    // Parametre de l'algorythme
    int N_outliers;


    AContractorPtr contractor;


    void contractCircle(interval &x0, interval &y0, interval &x1, interval &y1, interval &d);
    void contractCircle(interval& x0,interval& y0, double x1, double y1, interval& d);


    void Incremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double vit, double noise);
    void Incremente(box &X, box &X0, double theta, double vit, double err);
    void Decremente(box &X, box &X0, double theta, double vit, double err);
    void Decremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double V, double err);

    void runAll(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);
    void outerContractAll2(box &X, iMatrix &distances);
    void fixPoint(box &X, iMatrix &distance);
    void innerContract(box &X, int r);
    void outerContract(box &X, int r);
    void outerContractAll(box &X);
    void contractOneRobot(box &X, vector<box> &P, interval *distances, int robotNumber, bool direction);

    box vector2box(vector<box> &T);
    void runAll2(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);
    void Ctrajectory(box &X, vector<Robot *> *rob);
    vector<box> box2vector(box X, int boxSize);
    vector<int> findOutliers(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);
};

#endif // SIVIA_H
