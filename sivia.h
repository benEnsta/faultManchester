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
enum {ROB1, ROB2, ROB3, ROBALL};

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

    interval dist[4][4];
    vector<box> result;
    int C;
    void getDistances(vector<robot*> &robs);

    void contractCircle(interval &x0, interval &y0, interval &x1, interval &y1, interval &d);
    void contractAll(box &X);
    void contractAll2(box &X);
    void contractRX(box &X, int r);

    void contractR1(box &X);
    void contractR2(box &X);

    box getResult();
protected:

signals:

    void drawBox(box X,int type);
    void drawRobot(box X);
    void workFinished();
public slots:
    void doWork(box X0);
};

#endif // SIVIA_H
