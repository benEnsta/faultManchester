#include "mainwindow.h"
#include "ui_mainwindow.h"

double t = 0;
double dt = 0.05;
double u1 = 0, u2 = 0;
double xmin,xmax,ymin,ymax;
double epsilon;
box Xest(5);
//double xv=1.6,yv=0.3,thetav=0;
QPen color[] = {QPen(Qt::blue),QPen(Qt::green),QPen(Qt::red), QPen(Qt::yellow), QPen(QColor(100,0,200)),QPen(QColor(100,20,100)),QPen((QColor(40,200,200)))};
QBrush brushs[] = {QBrush(Qt::blue),QBrush(Qt::green),QBrush(Qt::red), QBrush(Qt::yellow),QBrush(QColor(100,0,200)), QBrush(QColor(100,20,100)),QBrush(QColor(40,200,200))};
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // set up display
    float width = 15;
    xmin=-width;xmax=width;    ymin=-width;ymax=width;
//    xmin=-13;xmax=1;    ymin=-13;ymax=1;
    //xmin=-1.2;xmax=2.8;    ymin=2.2;ymax=5.8;
    Rsivia=new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
    Rworld=new repere(this,ui->graphicsView_World,xmin,xmax,ymin,ymax);
    sivia = new SIVIA();
    connect(sivia,SIGNAL(drawBox(box,int)),this,SLOT(drawBox(box,int)));
    connect(sivia,SIGNAL(drawCircle( int)),this,SLOT(drawCircle( int)));

    sivia->N_outliers = 0;
    sivia->epsilon = ui->EpsilonSpinBox->value();

    // ---------- TIMER -----------
//    timer = new QTimer(this);
//    connect(timer,SIGNAL(timeout()),this, SLOT(clock()));
    // ---------- Add 3 robots --------------
    rob.push_back(new Robot());
    ui->selectRob->addItem("Robot 0");
    rob.push_back(new Robot(1,5, M_PI_2));
    ui->selectRob->addItem("Robot 1");
    rob.push_back(new Robot(-4,2));
    ui->selectRob->addItem("Robot 2");
    rob.push_back(new Robot(-2.5,-4.5,-M_PI_2));
    ui->selectRob->addItem("Robot 3");
//    rob.push_back(new Robot(-2.5,3.5));
//    ui->selectRob->addItem("Robot 4");
//    rob.push_back(new Robot(-4.5,-4.7));
//    ui->selectRob->addItem("Robot 5");
//    rob.push_back(new Robot(4.5,-3.5));
//    ui->selectRob->addItem("Robot 6");
    ui->selectRob->setCurrentIndex(0);

    ui->spinBox->setMaximum(rob.size()-1);
    drawRobots();
    Rworld->Save("SituationInitial.png");
}

MainWindow::~MainWindow()
{
    delete ui;
    for(uint i = 0; i < rob.size(); i++){
        delete rob[i];
    }
    delete sivia;
}


//======================================================================================================
//======================================================================================================
//======================================================================================================
//======================================================================================================

void MainWindow::drawBox(box X, QPen pen){
    for(uint i = 1; i < rob.size(); i++){
        Rworld->DrawBox(X[2*i+1].inf,X[2*i+1].sup,X[2*i+2].inf,X[2*i+2].sup,pen,QBrush(Qt::NoBrush));
    }
}

void MainWindow::drawBox(box X, int type)
{
    static int box = 0;
    // qDebug("receive Box");
    switch(type){

    case INSIDEBOX:
        Rworld->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::magenta),QBrush(Qt::red));
        break;
    case OUTSIDEBOX:
        Rworld->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::darkBlue),QBrush(Qt::NoBrush));
        break;
    case YELLOWBOX:
        Rworld->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::yellow),QBrush(Qt::NoBrush));
        break;
    case DARKBOX:
        Rworld->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::darkRed),QBrush(Qt::darkRed));
        break;
    case DARKBLUEBOX:
        Rworld->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::darkBlue),QBrush(Qt::NoBrush));
        break;
    case 99:
        for(uint i = 1; i < rob.size(); i++){
            Rworld->DrawBox(X[2*i+1].inf,X[2*i+1].sup,X[2*i+2].inf,X[2*i+2].sup,QPen(Qt::darkCyan),QBrush(Qt::NoBrush));
        }
    case 98:
        for(uint i = 1; i < rob.size(); i++){
            Rworld->DrawBox(X[2*i+1].inf,X[2*i+1].sup,X[2*i+2].inf,X[2*i+2].sup,QPen(Qt::yellow),QBrush(Qt::NoBrush));
        }
        break;
    //Rsivia->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,pen, brush);
    }
}

void MainWindow::drawCircle(int pos)
{
    int num_rob = ui->selectRob->currentIndex();
    vector< vector< interval> > dists = distances.at(pos);
    for(uint i = 0; i < rob.size(); i++){
        for(uint j = num_rob; j < num_rob+1/*rob.size()*/; j++){
            if(i == j) continue;
            Rworld->DrawEllipse(rob[i]->x_v[pos],rob[i]->y_v[pos],dists[j][i].inf,color[i],QBrush(Qt::NoBrush));
            Rworld->DrawEllipse(rob[i]->x_v[pos],rob[i]->y_v[pos],dists[j][i].sup,color[i],QBrush(Qt::NoBrush));
        }
    }
}

void MainWindow::drawRobots(){
    for(uint i = 0; i < rob.size(); i++){
        Rworld->DrawRobot(rob[i]->x,rob[i]->y,rob[i]->theta,0.2,color[i], brushs[i]);
    }
}

void MainWindow::drawDistances(uint k, interval *dist){
    //sivia->getDistances(rob);

    for(uint i = 0; i < rob.size(); i++){
        if(i == k) continue;
        Rworld->DrawEllipse(rob[i]->x,rob[i]->y,dist[i].inf,color[i],QBrush(Qt::NoBrush));
        Rworld->DrawEllipse(rob[i]->x,rob[i]->y,dist[i].sup,color[i],QBrush(Qt::NoBrush));
    }
}

//======================================================================================================
//======================================================================================================
//======================================================================================================
//======================================================================================================
void MainWindow::on_clearButtton_clicked()
{
    Rsivia->Clean();
    Rworld->Clean();
    drawRobots();
}


//void MainWindow::on_testContract2_clicked()
//{
//    box X0(2*rob.size());
//    srand(0);
//    // Initialize box
//    double err[] = {0.05, 0.5, 0.5, 0.3,0.2,0.2,0.5};
//    for(uint i = 0; i < rob.size(); i++){
//        X0[2*i+1] = interval(rob[i]->x,rob[i]->x) + interval(-err[i], err[i]);
//        X0[2*i+2] = interval(rob[i]->y,rob[i]->y) + interval(-err[i], err[i]);
//    }
//    //int indice  = ui->selectRob->currentIndex();
//    box tmp(X0[3],X0[4]);

//    //X0[3] = interval(xmin, xmax);
//    //X0[4] = interval(ymin, ymax);

//    // Initialize distances
//    vector< vector<interval> > dist(rob.size(),vector<interval> (rob.size()));
//    for(uint i = 0; i < rob.size(); i++){
//        for(uint j = 0; j < rob.size(); j++){
//            if(i == j) continue;
////            if(i == 2 && j == 1)
////                dist[i][j] = interval(rob[i]->getDistanceTo(rob[j]->x, rob[j]->y)) + interval(+2,+3);
////            else
//            // add noise
//            double trueDistance = rob[i]->getDistanceTo(rob[j]->x, rob[j]->y);
//            int noise_rd =  rand();
//            double noise = -0.1 + 0.2*noise_rd/(RAND_MAX);
////            //qDebug() << noise_rd << "  " << -0.1 + 0.2*((double) noise_rd/(RAND_MAX));
//                dist[i][j] = interval(trueDistance + noise) + interval(-0.1,0.1);
//        }
//    }
//    //exit(-1);
//    sivia->epsilon = ui->EpsilonSpinBox->value();
//     qDebug() <<  sivia->doContractOneByOne(X0,&dist);

//    Rworld->Save("ImageR2");
//    Rworld->SaveSVG("Test.svg");
//    //drawBox(tmp,QPen(Qt::green));
//    //Rworld->DrawBox(tmp[1].inf, tmp[1].sup,tmp[2].inf, tmp[2].sup, QPen(Qt::black), QBrush(Qt::NoBrush));
//}


// This function generate the true trajectorie for all robots and mesurements with noise



void MainWindow::generateData(int nb0){
    int n = 250;
    // Generate and draw robot's trajectory
    for(uint i = 0; i < rob.size(); i++){
        Robot* r = rob[i];
        r->generate8(i+1,n);
        for(uint i = 1; i < r->x_v.size(); i++){
            Rworld->DrawLine(r->x_v[i-1], r->y_v[i-1],r->x_v[i], r->y_v[i],QPen(Qt::black));
            if( i%10 == 0){
                Rworld->DrawRobot(r->x_v[i],r->y_v[i],r->theta_v[i],0.1);
            }
        }
    }
    qDebug() << "nb step " << n;
    for(uint i = 0; i < n; i++){
        iMatrix dist(rob.size(),vector<interval> (rob.size()));
        for(uint k = 0; k < rob.size(); k++){
            for(uint j = 0; j < rob.size(); j++){
                if(k == j) continue;
                double trueDistance = hypot(rob[k]->x_v[i] - rob[j]->x_v[i], rob[k]->y_v[i] - rob[j]->y_v[i]);
                dist[k][j] = interval(trueDistance) + interval(-0.1,0.1);
            }
        }
        distances.push_back(dist);
    }
}




//void MainWindow::clock(){
//    on_stepR1_clicked();
//}


void MainWindow::keyPressEvent(QKeyEvent *event)
{   switch (event->key())
    {
    case Qt::Key_A: u1=u1+0.1;          break;
    case Qt::Key_E: u1=u1-0.1;          break;
    case Qt::Key_Z: u2=u2+0.1;          break;
    case Qt::Key_S: u2=u2-0.1;          break;
    }
}


//void MainWindow::on_startTimer_clicked()
//{
//    if(!timer->isActive()){
//        ui->startTimer->setText("Stop Timer");
//        timer->start(100);
//        //Xest = box(rob.size());
//        for(uint i = 0; i < rob.size(); i++){
//            rob[i]->vit = 1;
//            Xest[2*i+1] = interval(rob[i]->x,rob[i]->x) + interval(-0.05, 0.05);
//            Xest[2*i+2] = interval(rob[i]->y,rob[i]->y) + interval(-0.05, 0.05);
//        }

//    } else {
//        ui->startTimer->setText("Start Timer");
//        timer->stop();
//    }
//}

//void MainWindow::on_contractState_clicked()
//{


//    box X0(ui->nbStep->value()*2*rob.size());

//    for(uint i = 1; i < X0.dim+1; i+= 1){
//        X0[i] = interval(-oo, +oo);
//    }

//    for(uint i = 0; i < rob.size(); i++){
//        X0[2*i+1] = interval(rob[i]->x) + interval(-0.001, 0.001);
//        X0[2*i+2] = interval(rob[i]->y) + interval(-0.001, 0.001);
//    }

//    generateData(ui->nbStep->value());

//    sivia->epsilon = ui->EpsilonSpinBox->value();

//    sivia->cont.clear();
//    sivia->incr.clear();
//    sivia->doContractState(X0,&rob, &distances);

//    if(sivia->cont.size() != sivia->incr.size() && sivia->cont.size() != distances.size()){
//        qDebug()<< "erreur taille result";
//    }

//    ui->resultBar->setMaximum(sivia->cont.size()-1);
//    ui->resultBox->setMaximum(sivia->cont.size()-1);

//    ui->contractState->setEnabled(false);
//    Rworld->Save("ImageALL");
//    for(int i = 1; i < sivia->cont.size(); i++){
//        box Xro(2*rob.size());
//        for(int  j = 0; j <  rob.size(); j++){
//            Robot *r = rob[j];
//            Xro[2*j+1] = interval(r->x_v[i]) + interval(-0.001,0.001);
//            Xro[2*j+2] = interval(r->y_v[i]) + interval(-0.001,0.001);
//        }
//        box Xr  = Inter(sivia->incr[i],Xro);
//        qDebug() << sivia->cont[i] << Xro;
//        if(Xr.IsEmpty()) qDebug("boite non consistance %d",i); qDebug() << Inter(sivia->cont[i],Xro);
//    }
//    //Rworld->SaveSVG("All.svg");
//}

//void MainWindow::on_drawALL_clicked()
//{
//    for(int i = 0; i < distances.size(); i++){
//        drawCircle(i);
//    }
//}


void MainWindow::on_spinBox_editingFinished()
{
    sivia->N_outliers = ui->spinBox->value();
}

void MainWindow::on_EpsilonSpinBox_editingFinished()
{
    sivia->epsilon = ui->EpsilonSpinBox->value();
}

//void MainWindow::on_stepR1_clicked()
//{
//    Rworld->Clean();
//    Rsivia->Clean();
//    box X0(2*rob.size());
//    if(sivia->reccordNumber == 0){
//        // initialisation de l'etat
//        double epss[] = {0.001, 0.001, 0.001, 0.001};
//        for(uint i = 0; i < rob.size(); i++){
//            double error = epss[i];
//            X0[2*i+1] = interval(rob[i]->x_v[sivia->reccordNumber]) + interval(-error, error);
//            X0[2*i+2] = interval(rob[i]->y_v[sivia->reccordNumber]) + interval(-error, error);
//        }
//        Xc = X0;
//        sivia->reccordNumber++;
//    } else {
//        box Xk(2*rob.size());
//        double epss[] = {0.0, 0.01, 0.01, 0.01};
//        for(uint i = 0; i < rob.size(); i++){
//            Xk[2*i+1] = interval(-oo, oo);
//            Xk[2*i+2] = interval(-oo, oo);
//            double error = epss[i];
//            sivia->Incremente(Xk[2*i+1],Xk[2*i+2],Xc[2*i+1],Xc[2*i+2],rob[i]->theta_v[sivia->reccordNumber],rob[i]->speed_v[sivia->reccordNumber],error);
//        }
//        Xc = Xk;
//    }


//    drawBox(Xc,5);
//    drawCircle(sivia->reccordNumber);
//    for(uint i = 0; i < rob.size(); i++){
//        Rworld->DrawRobot(rob[i]->x_v[sivia->reccordNumber],rob[i]->y_v[sivia->reccordNumber],rob[i]->theta_v[sivia->reccordNumber],0.2,color[i], brushs[i]);
//    }
//    sivia->epsilon = ui->EpsilonSpinBox->value();
//    if(Xc.Width() < 0.1) Inflate(Xc,0.1);
//    //sivia->doWork(Xc, &distances[sivia->reccordNumber]);
//    box Xr = sivia->getResult();
//    if (!Xr.IsEmpty()){
//        //Xc = Inter(Xc,Xr);
//    }


//    qDebug() << sivia->reccordNumber;
////    Rworld->centerOn(rob[1]->x_v[sivia->reccordNumber],rob[1]->y_v[sivia->reccordNumber],0.5,0.5);
//    if(sivia->reccordNumber < distances.size()-1){
//        sivia->reccordNumber++;
//    }
//}

void MainWindow::on_DrawCircleBtn_clicked()
{
    int indice = ui->selectRob->currentIndex();
    vector<interval> dist(rob.size());
    for(uint j = 0; j < rob.size(); j++){
        if(indice == j) continue;
        if(sivia->dist.size() > 0){
            dist[j] = sivia->dist[indice][j];
        } else
            dist[j] = interval(rob[indice]->getDistanceTo(rob[j]->x, rob[j]->y)) + interval(-0.1,0.1);
    }
    drawDistances(indice, &dist[0]);
    //drawCircle(indice);
    dist.clear();
}



void MainWindow::on_runTestBtn_clicked()
{
    generateData(100);



//    box X0(2*rob.size());
//    //double eps = 0.1;
//    double err[] = {0.05, 0.1, 0.5, 0.5, 0.1, 0.1, 0.1};
//    for(uint i = 0; i < rob.size(); i++){
//        X0[2*i+1] = interval(rob[i]->x,rob[i]->x) + interval(-err[i], err[i]);
//        X0[2*i+2] = interval(rob[i]->y,rob[i]->y) + interval(-err[i], err[i]);
//    }
//    int indice  = ui->selectRob->currentIndex();
//    X0[2*indice+1] = interval(xmin, xmax);
//    X0[2*indice+2] = interval(ymin, ymax);



//    vector< vector<interval> > dist(rob.size(),vector<interval> (rob.size()));
//    for(uint i = 0; i < rob.size(); i++){
//        for(uint j = 0; j < rob.size(); j++){
//            if(i == j) continue;
//            dist[i][j] = interval(rob[i]->getDistanceTo(rob[j]->x, rob[j]->y)) + interval(-0.1,0.1);
//            //if(j == 3){
//            //    dist[i][j] = interval(rob[i]->getDistanceTo(rob[j]->x, rob[j]->y)) + interval(-2,-1.5);
//            //}
//        }

//    }

//    sivia->doWork(X0,indice, &dist);
//    Rworld->Save("ImageR2");
//    Rworld->SaveSVG("ImageR2");
}

//void MainWindow::on_dynLocBtn_clicked()
//{

//    box X0(2*rob.size());
//    //double eps = 0.1;
//    double err[] = {0.001, 0.5, 0.5, 0.5, 0.5,0.2,0.1,0.2};
//    for(uint i = 0; i < rob.size(); i++){
//        X0[2*i+1] = interval(rob[i]->x,rob[i]->x) + interval(-err[i], err[i]);
//        X0[2*i+2] = interval(rob[i]->y,rob[i]->y) + interval(-err[i], err[i]);
//    }


//    vector< vector<interval> > dist(rob.size(),vector<interval> (rob.size()));
//    for(uint i = 0; i < rob.size(); i++){
//        for(uint j = 0; j < rob.size(); j++){
//            if(i == j) continue;
//            dist[i][j] = interval(rob[i]->getDistanceTo(rob[j]->x, rob[j]->y)) + interval(-0.1,0.1);
//        }
//    }
//    drawBox(X0,QPen(Qt::darkGreen));


//    //sivia->C = ROBALL;
//    sivia->doAllInOne(X0,&dist);
//    qDebug() << sivia->result.size();
//    sivia->getResult();
//    Rworld->Save("ImageR2");

//}

void MainWindow::on_resultBar_sliderMoved(int position)
{
}

void MainWindow::on_resultBar_valueChanged(int position)
{
    Rworld->Clean();
    Rsivia->Clean();
    int indice = ui->selectRob->currentIndex();
    drawBox(sivia->incr[position],QPen(Qt::green));
    drawBox(sivia->cont[position],QPen(Qt::darkBlue));
    drawCircle(position);
    for(uint i = 0; i < rob.size(); i++){
        Rworld->DrawRobot(rob[i]->x_v[position],rob[i]->y_v[position],rob[i]->theta_v[position],0.2,color[i], brushs[i]);
    }
    sivia->reccordNumber = position;
    ui->resultBox->setValue(position);
}


//void MainWindow::on_pushButton_clicked()
//{
//    Rworld->Clean();
//    Rsivia->Clean();
//    int position = ui->resultBar->value();
//    box X0 = sivia->incr[position];
//    for(uint i = 0; i < rob.size(); i++){
//        Rworld->DrawRobot(rob[i]->x_v[position],rob[i]->y_v[position],rob[i]->theta_v[position],0.2,color[i], brushs[i]);
//    }
//    int indice  = ui->selectRob->currentIndex();
//    X0[2*indice+1] = interval(xmin, xmax);
//    X0[2*indice+2] = interval(ymin, ymax);
//    drawCircle(position);
//    sivia->doContractOneByOne(X0, &distances.at(position));
//    //sivia->doWork(X0,indice, &distances.at(position));
//    if(indice == 2)
//        qDebug() << distances.at(position)[indice][5];
//     else if(indice == 5)
//        qDebug() << distances.at(position)[indice][2];
//    qDebug() << hypot(rob[5]->x_v[position]-rob[2]->x_v[position], rob[5]->y_v[position]-rob[2]->y_v[position]);
//}



void MainWindow::on_contractAll_clicked()
{

    int step = distances.size();
    vector<box> T0(step,box(interval(-oo,oo), 2*rob.size()));

    for(uint i = 0; i < rob.size(); i++){
        T0[0][2*i+1] = interval(rob[i]->x_v[0]) + interval(-0.001, 0.001);
        T0[0][2*i+2] = interval(rob[i]->y_v[0]) + interval(-0.001, 0.001);
    }


    sivia->epsilon = ui->EpsilonSpinBox->value();

    sivia->cont.clear();
    sivia->incr.clear();
    sivia->runAll(T0,&rob,distances);

    for(uint i = 0; i < T0.size(); i++){
//        int j = 3;
        for(uint j = 0; j < rob.size(); j++){
            Rworld->DrawBox(T0[i][2*j+1].inf,T0[i][2*j+1].sup,T0[i][2*j+2].inf,T0[i][2*j+2].sup,QPen(Qt::yellow),QBrush(Qt::NoBrush));
        }
    }
//    sivia->doContractState(X0,&rob, &distances);

//    if(sivia->cont.size() != sivia->incr.size() && sivia->cont.size() != distances.size()){
//        qDebug()<< "erreur taille result";
//    }

//    ui->resultBar->setMaximum(sivia->cont.size()-1);
//    ui->resultBox->setMaximum(sivia->cont.size()-1);

//    ui->contractState->setEnabled(false);
//    Rworld->Save("ImageALL");
//    for(int i = 1; i < sivia->cont.size(); i++){
//        box Xro(2*rob.size());
//        for(int  j = 0; j <  rob.size(); j++){
//            Robot *r = rob[j];
//            Xro[2*j+1] = interval(r->x_v[i]) + interval(-0.001,0.001);
//            Xro[2*j+2] = interval(r->y_v[i]) + interval(-0.001,0.001);
//        }
//        box Xr  = Inter(sivia->incr[i],Xro);
//        qDebug() << sivia->cont[i] << Xro;
//        if(Xr.IsEmpty()) qDebug("boite non consistance %d",i); qDebug() << Inter(sivia->cont[i],Xro);
//    }
    //Rworld->SaveSVG("All.svg");
}
