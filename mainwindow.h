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
    

    void keyPressEvent(QKeyEvent *event);
    void generateData(int nb);



    void drawRobots(int step = 0);
    void drawAllTrajectories();
    void drawTrajectory(int robot_num);
    void generateDistances(int nb0);
    void checkIntegrity(vector<box> &T0);
    void drawCircles(int num_rob, int pos);

    void update_interface();
    void drawBoxesState(int step);
private:
    vector<box> T0;
private slots:


    void on_clearButtton_clicked();
//    void on_testContract2_clicked();

//    void clock();

//    void on_drawALL_clicked();

//    void on_stepR1_clicked();

//    void on_DrawCircleBtn_clicked();

    void on_runTestBtn_clicked();

//    void on_dynLocBtn_clicked();


    void on_resultBar_valueChanged(int position);

    void on_spinBox_editingFinished();

    void on_EpsilonSpinBox_editingFinished();

//    void on_pushButton_clicked();

    void on_DrawCircleBtn_clicked();

    void on_contractAll_clicked();

    void on_resultBox_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;
    repere *Rsivia,*Rworld;
    SIVIA* sivia;
    vector<Robot*> rob;
    vector <vector <vector < interval> > > distances;
};

#endif // MAINWINDOW_H
