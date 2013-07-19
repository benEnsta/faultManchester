#include "mainwindow.h"
#include "ui_mainwindow.h"

double t = 0;
double dt = 0.05;
double u1 = 0, u2 = 0;
double xmin,xmax,ymin,ymax;
double epsilon;
//double xv=1.6,yv=0.3,thetav=0;
QPen color[4] = {QPen(Qt::blue),QPen(Qt::green),QPen(Qt::red), QPen(Qt::yellow)};
QBrush brushs[4] = {QBrush(Qt::blue),QBrush(Qt::green),QBrush(Qt::red), QBrush(Qt::yellow)};
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // set up display
    xmin=-5;xmax=5;    ymin=-5;ymax=5;
    Rsivia=new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Rworld=new repere(this,ui->graphicsView_World,xmin,xmax,ymin,ymax);
    sivia = new SIVIA();
    connect(sivia,SIGNAL(drawBox(box,int)),this,SLOT(drawBox(box,int)));

    // ---------- TIMER -----------
    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this, SLOT(clock()));
    // ---------- Add 3 robots --------------
    rob.push_back(new robot());
    rob.push_back(new robot(1,3));
    rob.push_back(new robot(-4,2));
    rob.push_back(new robot(-2.5,-1.5));
    drawRobots();

}

MainWindow::~MainWindow()
{
    delete ui;
    for(int i = 0; i < rob.size(); i++){
        delete rob[i];
    }
    delete sivia;
}


void MainWindow::drawBox(box X, int type)
{
    // qDebug("receive Box");
    switch(type){
    case 0:
        Rworld->DrawBox(X[3].inf,X[3].sup,X[4].inf,X[4].sup,QPen(Qt::green),QBrush(Qt::NoBrush));
        break;
    case 1:
        switch(sivia->C){
        case ROB1: Rworld->DrawBox(X[3].inf,X[3].sup,X[4].inf,X[4].sup,QPen(Qt::blue),QBrush(Qt::NoBrush)); break;
        case ROB2: Rworld->DrawBox(X[5].inf,X[5].sup,X[6].inf,X[6].sup,QPen(Qt::blue),QBrush(Qt::NoBrush)); break;
        case ROB3: Rworld->DrawBox(X[7].inf,X[7].sup,X[8].inf,X[8].sup,QPen(Qt::blue),QBrush(Qt::NoBrush)); break;
        case ROBALL:
            Rworld->DrawBox(X[3].inf,X[3].sup,X[4].inf,X[4].sup,QPen(Qt::blue),QBrush(Qt::NoBrush));
            Rworld->DrawBox(X[5].inf,X[5].sup,X[6].inf,X[6].sup,QPen(Qt::blue),QBrush(Qt::NoBrush));
            Rworld->DrawBox(X[7].inf,X[7].sup,X[8].inf,X[8].sup,QPen(Qt::blue),QBrush(Qt::NoBrush)); break;
            break;

        //Rworld->DrawRobot(Center(X[1]),Center(X[2]),Center(X[3]),0.1);
        }
        break;
    case 2:
        switch(sivia->C){
        case ROB1: Rworld->DrawBox(X[3].inf,X[3].sup,X[4].inf,X[4].sup,QPen(Qt::red),QBrush(Qt::NoBrush)); break;
        case ROB2: Rworld->DrawBox(X[5].inf,X[5].sup,X[6].inf,X[6].sup,QPen(Qt::red),QBrush(Qt::NoBrush)); break;
        case ROB3: Rworld->DrawBox(X[7].inf,X[7].sup,X[8].inf,X[8].sup,QPen(Qt::red),QBrush(Qt::NoBrush)); break;
        case ROBALL:
            Rworld->DrawBox(X[3].inf,X[3].sup,X[4].inf,X[4].sup,QPen(Qt::red),QBrush(Qt::NoBrush));
            Rworld->DrawBox(X[5].inf,X[5].sup,X[6].inf,X[6].sup,QPen(Qt::red),QBrush(Qt::NoBrush));
            Rworld->DrawBox(X[7].inf,X[7].sup,X[8].inf,X[8].sup,QPen(Qt::red),QBrush(Qt::NoBrush)); break;
            break;
        }
        break;

    }

    //Rsivia->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,pen, brush);
}

void MainWindow::drawRobots(){
    for(int i = 0; i < rob.size(); i++){
        Rworld->DrawRobot(rob[i]->x,rob[i]->y,rob[i]->theta,0.2,color[i], brushs[i]);
    }
}

void MainWindow::drawDistances(int k){
    sivia->getDistances(rob);

    for(int i = 0; i < rob.size(); i++){
        for(int j = k; j < k+1/*rob.size()*/; j++){
            if(i == j) continue;
            Rworld->DrawEllipse(rob[i]->x,rob[i]->y,sivia->dist[i][j].inf,color[i],QBrush(Qt::NoBrush));
            Rworld->DrawEllipse(rob[i]->x,rob[i]->y,sivia->dist[i][j].sup,color[i],QBrush(Qt::NoBrush));
        }
    }
}


void MainWindow::on_DrawDistanceR1_clicked()
{
    drawDistances(1);
}
void MainWindow::on_DrawDistanceR2_clicked()
{
    drawDistances(2);
}
void MainWindow::on_DrawDistanceR3_clicked()
{
    drawDistances(3);
}

void MainWindow::on_clearButtton_clicked()
{
    Rsivia->Clean();
    Rworld->Clean();
    drawRobots();
}


void MainWindow::on_testR1_clicked()
{
    box X0(8);
    X0[1] = interval(-0.001, 0.001);
    X0[2] = interval(-0.001, 0.001);
    X0[3] = interval(xmin, xmax);
    X0[4] = interval(ymin, ymax);

    //X0[3] = interval(rob[1]->x,rob[1]->x) + interval(-0.001, 0.001);
    //X0[4] = interval(rob[1]->y,rob[1]->y) + interval(-0.001, 0.001);


    //X0[5] = interval(xmin, xmax);
    //X0[6] = interval(ymin, ymax);
    X0[5] = interval(rob[2]->x,rob[2]->x) + interval(-0.001, 0.001);
    X0[6] = interval(rob[2]->y,rob[2]->y) + interval(-0.001, 0.001);

    X0[7] = interval(rob[3]->x,rob[3]->x) + interval(-0.001, 0.001);
    X0[8] = interval(rob[3]->y,rob[3]->y) + interval(-0.001, 0.001);
    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->getDistances(rob);
    sivia->C = ROB1;
    sivia->doWork(X0);
    //qDebug() << sivia->getResult();

}


void MainWindow::on_testR2_clicked()
{
    box X0(8);
    X0[1] = interval(-0.001, 0.001);
    X0[2] = interval(-0.001, 0.001);
    X0[3] = interval(rob[1]->x,rob[1]->x) + interval(-0.001, 0.001);
    X0[4] = interval(rob[1]->y,rob[1]->y) + interval(-0.001, 0.001);


    X0[5] = interval(xmin, xmax);
    X0[6] = interval(ymin, ymax);

    X0[7] = interval(rob[3]->x,rob[3]->x) + interval(-0.001, 0.001);
    X0[8] = interval(rob[3]->y,rob[3]->y) + interval(-0.001, 0.001);
    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->getDistances(rob);
    sivia->C = ROB2;
    sivia->doWork(X0);
    //qDebug() << sivia->getResult();
}



void MainWindow::on_testContract2_clicked()
{
    box X0(8);
    double eps = 1;
    X0[1] = interval(-0.001, 0.001);
    X0[2] = interval(-0.001, 0.001);
    //X0[3] = interval(rob[1]->x,rob[1]->x) + interval(-0.5, 0.5);
    //X0[4] = interval(rob[1]->y,rob[1]->y) + interval(-0.5, 0.5);
    X0[3] = interval(rob[1]->x,rob[1]->x) + interval(-eps, eps);
    X0[4] = interval(rob[1]->y,rob[1]->y) + interval(-eps, eps);
    X0[5] = interval(rob[2]->x,rob[2]->x) + interval(-eps, eps);
    X0[6] = interval(rob[2]->y,rob[2]->y) + interval(-eps, eps);

    //X0[5] = interval(rob[2]->x,rob[2]->x) + interval(-0.5, 0.5);
    //X0[6] = interval(rob[2]->y,rob[2]->y) + interval(-0.5, 0.5);

    X0[7] = interval(rob[3]->x,rob[3]->x) + interval(-eps, eps);
    X0[8] = interval(rob[3]->y,rob[3]->y) + interval(-eps, eps);
    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->getDistances(rob);
    sivia->C = ROBALL;
    sivia->doWork(X0);
    //qDebug() << sivia->getResult();
}

void MainWindow::on_testR3_clicked()
{
    box X0(8);
    X0[1] = interval(-0.001, 0.001);
    X0[2] = interval(-0.001, 0.001);
    X0[3] = interval(rob[1]->x,rob[1]->x) + interval(-0.001, 0.001);
    X0[4] = interval(rob[1]->y,rob[1]->y) + interval(-0.001, 0.001);


    X0[7] = interval(xmin, xmax);
    X0[8] = interval(ymin, ymax);

    X0[5] = interval(rob[2]->x,rob[2]->x) + interval(-0.001, 0.001);
    X0[6] = interval(rob[2]->y,rob[2]->y) + interval(-0.001, 0.001);
    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->getDistances(rob);
    sivia->C = ROB3;
    sivia->doWork(X0);
}


void MainWindow::clock(){
    Rworld->Clean();
    for(int i = 0; i < rob.size(); i++){

        //
        //Rworld->Scene->
        rob[i]->Clock(u1,u2);

    }
    //Rworld->Scene->setSceneRect(QRect(QPoint(rob[0]->x - 5,rob[0]->y - 5),
    //                                  QPoint(rob[0]->x + 5,rob[0]->y + 5)));
    t = t+ dt;
    qDebug(" t %f",t);
    drawRobots();

}
void MainWindow::keyPressEvent(QKeyEvent *event)
{   switch (event->key())
    {
    case Qt::Key_A: u1=u1+0.1;          break;
    case Qt::Key_E: u1=u1-0.1;          break;
    case Qt::Key_Z: u2=u2+0.1;          break;
    case Qt::Key_S: u2=u2-0.1;          break;
    }
}
void MainWindow::on_startTimer_clicked()
{
    if(!timer->isActive()){
        ui->startTimer->setText("Stop Timer");
        timer->start(100);
    } else {
        ui->startTimer->setText("Start Timer");
        timer->stop();
    }
}
