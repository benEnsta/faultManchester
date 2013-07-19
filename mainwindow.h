#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <robot.h>
#include "tools/repere.h"
#include "sivia.h"
#include <vector>
#include <QTimer>
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
    void drawDistances(int k);
    void keyPressEvent(QKeyEvent *event);
private slots:
    // SIVIA SLOT
    void drawBox(box X, int type);

    //BUTTONS SLOT
    void on_DrawDistanceR1_clicked();
    void on_DrawDistanceR2_clicked();
    void on_DrawDistanceR3_clicked();

    void on_testR1_clicked();
    void on_testR2_clicked();
    void on_testR3_clicked();

    void on_clearButtton_clicked();
    void on_testContract2_clicked();

    void clock();


    void on_startTimer_clicked();

private:
    Ui::MainWindow *ui;
    repere *Rsivia,*Rworld;
    SIVIA* sivia;
    vector<robot*> rob;

    QTimer *timer;
};

#endif // MAINWINDOW_H
