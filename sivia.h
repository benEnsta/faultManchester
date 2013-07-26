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

using namespace std;
enum {ROB1, ROB2, ROB3, ROBALL, STATE, STEPR1};

class SIVIA : public QObject
{
    Q_OBJECT
public:
    explicit SIVIA();
    ~SIVIA();
    void run();

    // Parametre de l'algorythme
    int N_outliers;
    float epsilon;

    void Init();
    int SIVIA_f(box X0);
 //   void contractCircle(interval &x0, double y0, interval &x1, double y1, interval &d);

    vector< vector <interval> > dist;
    vector<box> result;
    vector<box> result1;
    vector<robot*> *rob;
    vector< vector< vector<interval> > > *distance;

    int m;
    int reccordNumber;
    int C;
    void getDistances(vector<robot*> &robs);

    void contractCircle(interval &x0, interval &y0, interval &x1, interval &y1, interval &d);
    void contractCircle(interval& x0,interval& y0, double x1, double y1, interval& d);
    void contractAll(box &X);
    void contractAll2(box &X);
    void contractAt(box& X, vector < vector < interval> > &dists);
    void contractRX(box &X, int r);

    void contractR1(box &X);
    void contractR2(box &X);

    box getResult();
    void stateEstim(box &X, vector<robot *> &robs);
    void Incremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double vit, double noise);
    void Incremente(box &X0, box X, double theta, double vit);
    void Decremente(box &X0, box X, double theta, double vit);
    void Decremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double V);
    void innerContract(box &X, int r);
    void contract_and_draw(box &X, int r);
    void outerContract(box &X, int r);
    void contractInOut(box &X);
    void contractState(box &X);
    void doContractState(box X0, vector<robot *> *rob, vector<vector<vector<interval> > > *distance);
    void contractInOut(box &X, vector<vector<interval> > distance);
    void contractReccord(box &X);
    void doStepR1(box X0, vector<robot *> *rob, vector<vector<vector<interval> > > *distance);
protected:

signals:

    void drawBox(box X,int type);
    void drawCircle(vector< vector< interval > > dists, int pos);
    void drawRobot(box X);
    void workFinished();
public slots:
    void doWork(box X0);
};

#endif // SIVIA_H
