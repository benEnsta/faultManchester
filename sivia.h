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
enum {YELLOWBOX, DARKBOX, INSIDEBOX, OUTSIDEBOX, DARKBLUEBOX};
typedef vector <vector < interval> > iMatrix;
class SIVIA : public QObject
{
    Q_OBJECT
public:
    explicit SIVIA();

     typedef void (SIVIA::*AContractorPtr)(box &);
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
    vector<box> incr;
    vector<box> cont;
    vector<Robot*> *rob;
    vector<box> P0;
    int currentRob;

    vector< vector< vector<interval> > > *distance;

    AContractorPtr contractor;
    int m;
    int reccordNumber;
    void getDistances(vector<Robot*> &robs);

    void contractCircle(interval &x0, interval &y0, interval &x1, interval &y1, interval &d);
    void contractCircle(interval& x0,interval& y0, double x1, double y1, interval& d);
    void contractAll(box &X);
    void contractAt(box& X, vector < vector < interval> > &dists);


    box getResult();
    void stateEstim(box &X, vector<Robot *> &robs);
    void Incremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double vit, double noise);
    void Incremente(box &X0, box X, double theta, double vit);
    void Decremente(box &X0, box X, double theta, double vit);
    void Decremente(interval &X1, interval &Y1, interval &X, interval &Y, double theta, double V);
    void innerContract(box &X, int r);
    void contract_and_draw(box &X);
    void outerContract(box &X, int r);
    void contractInOut(box &X);
    void contractState(box &X);
    void doContractState(box X0, vector<Robot *> *rob, vector<vector<vector<interval> > > *distance);
    void contractInOut(box &X, vector<vector<interval> > distance);
    void contractReccord(box &X);
    void doStepR1(box X0, vector<Robot *> *rob, vector<vector<vector<interval> > > *distance);
    void outerContractAll(box &X);
    void innerContractAll(box &X);
    box doContractOneByOne(box X, vector<vector<interval> > *dist);

    void contractOneRobot(box &X, vector<box> &P, interval *distances, int robotNumber, bool direction);
    void wrapperContractOneRobot(box &X);

    void contract_and_draw(box &X, AContractorPtr contract, bool outside);
    void innerContractOneRobot(box &X);
    void outerContractOneRobot(box &X);
    void contract_and_drawOneRobot(box &X);
    void doAllInOne(box X0, vector<vector<interval> > *distance);
    void runAll(vector<box> &T0, vector<Robot *> *rob, vector<iMatrix> &distance);
    void outerContractAll2(box &X, iMatrix &distances);
    void fixPoint(box &X, iMatrix &distance);
protected:

signals:

    void drawBox(box X,int type);
    void drawCircle(int pos);
    void drawRobot(box X);
    void workFinished();
public slots:
    void doWork(box X0, int robotNumber, vector<vector<interval> > *dist);
};

#endif // SIVIA_H
