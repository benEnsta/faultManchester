/**
 * \file      mainwindow.h
 * \author    Benoit DESROCHERS <benoitdesrochers@ensta-bretagne.org>
 * \version   1.0
 * \date      mar 24, 2014
 * \brief     Gui header interface
 */

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
    




    void runLocalisation();
    void generateData(int steps_per_tour, int numberOfStep);
    void generateDistancesWithRandomOutliers(int nb0);
    void generateDistancesWithBrokenSensor(int nb0, int robNumber, int intialStep, int finalStep);
    void checkIntegrity(vector<box> &T0);





    // DRAWING FUNCTIONS
    void drawRobots(int step = 0);
    void drawAllTrajectories();
    void drawTrajectory(int robot_num, int tmax);
    void drawBoxesState(int step);
    void drawCircles(int num_rob, int pos);



    void drawOutliers(int tmax);
    void exportResults(const vector<box> &T);
    void logRobot(int robNum);
    void generateDistancesWith2BrokenSensors(int nb0);
    void generateDistancesWithoutOutliers(int nbSteps);
    void breakSensor(int robNumber, int intialStep, int finalStep);
private:
    vector<box> T0;
private slots:

    // GUI FUNCTIONS
    void on_clearButtton_clicked();
    void on_runTestBtn_clicked();
    void on_resultBar_valueChanged(int position);
    void on_N_outliers_editingFinished();

    void on_contractAll_clicked();
    void on_resultBox_valueChanged(int arg1);
    void on_drawAllButton_clicked();
    void update_interface();
    void keyPressEvent(QKeyEvent *event);




    void on_drawOneButton_clicked();

private:
    Ui::MainWindow *ui;
    repere *Rworld;
    SIVIA* sivia;
    vector<Robot*> rob;
    vector <iMatrix> distances;
    vector<int> outliers;
};

#endif // MAINWINDOW_H
