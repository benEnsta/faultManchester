#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <robot.h>
#include "tools/repere.h"
#include "sivia.h"
#include <vector>
#include <QTimer>
#include <stdlib.h>
using namespace std;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
    void drawRobots();
    void drawDistances(uint k);
    void keyPressEvent(QKeyEvent *event);
    void generateData(int nb);
    void drawDistances(uint k, interval *dist);
    void drawBox(box X, QPen pen);


private slots:
    // SIVIA SLOT
    void drawBox(box X, int type);
    void drawCircle(int pos);
    //BUTTONS SLOT

    void on_clearButtton_clicked();
//    void on_testContract2_clicked();

//    void clock();

//    void on_drawALL_clicked();

//    void on_stepR1_clicked();

//    void on_DrawCircleBtn_clicked();

    void on_runTestBtn_clicked();

//    void on_dynLocBtn_clicked();

    void on_resultBar_sliderMoved(int position);

    void on_resultBar_valueChanged(int position);

    void on_spinBox_editingFinished();

    void on_EpsilonSpinBox_editingFinished();

//    void on_pushButton_clicked();

    void on_DrawCircleBtn_clicked();

    void on_contractAll_clicked();

private:
    Ui::MainWindow *ui;
    repere *Rsivia,*Rworld;
    SIVIA* sivia;
    vector<Robot*> rob;
    vector <vector <vector < interval> > > distances;
    vector<box> pos;
    box Xc;
    QTimer *timer;
};

#endif // MAINWINDOW_H
