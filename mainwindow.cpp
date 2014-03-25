#include "mainwindow.h"
#include "ui_mainwindow.h"


double t = 0;
double dt = 0.05;
double u1 = 0, u2 = 0;
double xmin,xmax,ymin,ymax;


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

    sivia->N_outliers = 0;
    sivia->epsilon = ui->EpsilonSpinBox->value();


    // ---------- Add 3 robots --------------
    rob.push_back(new Robot());
    ui->selectRob->addItem("Robot 0");

    rob.push_back(new Robot(8,7, -M_PI_4, 0.01));
    ui->selectRob->addItem("Robot 1");

    rob.push_back(new Robot(-10,-2, 0, 0.01));
    ui->selectRob->addItem("Robot 2");

    rob.push_back(new Robot(-5,-9,-M_PI_2, 0.000));
    ui->selectRob->addItem("Robot 3");

    rob.push_back(new Robot(-5,7, 0.3, 0.01));
    ui->selectRob->addItem("Robot 4");
//    rob.push_back(new Robot(-4.5,-4.7,-M_PI_4, 0.01));
//    ui->selectRob->addItem("Robot 5");
//    rob.push_back(new Robot(4.5,-3.5,M_PI_4, 0.01));
//    ui->selectRob->addItem("Robot 6");
    ui->selectRob->setCurrentIndex(1);
    ui->nbStep->setValue(100);
//    ui->N_outliers->setMaximum(rob.size()-1);
    ui->resultBar->setMaximum(0);
    ui->resultBox->setMaximum(0);
    drawRobots();
    Rworld->Save("SituationInitial.png");
    srand(0);
}

MainWindow::~MainWindow()
{
    delete ui;
    for(uint i = 0; i < rob.size(); i++){
        delete rob[i];
    }
    delete sivia;
}


double generateRandomOutliers(int i, int j, int k, double dist){
    int noise_rd =  rand();
    if( ((double) noise_rd/RAND_MAX) > 0.99){
        dist *= 10;
        qDebug() << "outlier with " << i << " " << k << " "<< j ;
    }
//                if(i == 50 && j == 1 && k == 2){
//                    trueDistance = 1000;
//                }

    return dist;
}

// return the distance between the robot number n1 and n2 .
void MainWindow::generateDistancesWithBrokenSensor(int nb0, int robNumber, int timeStep){
    int nOut = 0;
    for(uint i = 0; i < nb0; i++){ // for each time step
        iMatrix dist(rob.size(),vector<interval> (rob.size()));
        for(uint k = 0; k < rob.size(); k++){ // for each pair of robot i,j
            for(uint j = 0; j < rob.size(); j++){
                if(k == j) continue;
                double trueDistance = hypot(rob[k]->x_v[i] - rob[j]->x_v[i], rob[k]->y_v[i] - rob[j]->y_v[i]);

                double noise = -0.09 + 0.18*rand()/(RAND_MAX);
                dist[k][j] = interval(trueDistance + noise) + interval(-0.1,0.1);
            }
        }
        distances.push_back(dist);


    }

    for(uint i = timeStep; i < distances.size(); i++){
        for(uint j = 0; j < rob.size(); j++){
            if(i == j) continue;
            distances[i][robNumber][j] = interval(oo);
        }
    }
    qDebug() << "noutlier"  << nOut;
}

//-----------------------------------------------------------------------------------------------
// return the distance between the robot number n1 and n2 with a noise.
// Outlier generation could be added in this part
void MainWindow::generateDistancesWithRandomOutliers(int nb0){
    int nOut = 0;
    for(uint i = 0; i < nb0; i++){ // for each time step
        iMatrix dist(rob.size(),vector<interval> (rob.size()));
        for(uint k = 0; k < rob.size(); k++){ // for each pair of robot i,j
            for(uint j = 0; j < rob.size(); j++){
                if(k == j) continue;
                double trueDistance = hypot(rob[k]->x_v[i] - rob[j]->x_v[i], rob[k]->y_v[i] - rob[j]->y_v[i]);
                double distance = trueDistance;
                if( rand() > 0.99*RAND_MAX){
                    distance *= 1.1;
                    qDebug() << "outlier" << i << " " << k << " " << j;
                }
                if( distance != trueDistance) nOut++;
                int noise_rd =  rand();
                double noise = -0.09 + 0.18*noise_rd/(RAND_MAX);
                dist[k][j] = interval(distance + noise) + interval(-0.1,0.1);

            }
        }
        distances.push_back(dist);
    }
    qDebug() << "noutlier"  << nOut;
}

//--------------------------------------------------------------------------------------------------
// Generate trajectories for all robot and all distances between them
void MainWindow::generateData(int nb0){

    // Clean previous datas
    distances.clear();
    // Generate robot's trajectory
    for(uint i = 0; i < rob.size(); i++){
        Robot* r = rob[i];
        r->generate8(1.25*i+1,nb0);
    }

    if(ui->BrokenSensor->isChecked())
        generateDistancesWithBrokenSensor(nb0,ui->selectRob->currentIndex(),30);
    else
        generateDistancesWithRandomOutliers(nb0);
    drawAllTrajectories();
}



void MainWindow::logRobot(int robNum){
    QString filename("./log/");
    filename += QString::number(robNum) + ".txt";
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
           return;
    QTextStream out(&file);

    int i = robNum;
    // lower bound of [x] - x_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(T0[j][2*i+1].inf-rob[i]->x_v[j]) << " ";
    } out << "\n";
    // upper bound of [x] - x_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(T0[j][2*i+1].sup - rob[i]->x_v[j]) << " ";
    } out << "\n";
    // lower bound of [y] - y_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(T0[j][2*i+2].inf -rob[i]->y_v[j]) << " ";
    } out << "\n";
    // upper bound of [y] - y_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(T0[j][2*i+2].sup - rob[i]->y_v[j]) << " ";
    } out << "\n";
    // x_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(rob[i]->x_v[j]) << " ";
    } out << "\n";

    // y_ref
    for(uint j = 0; j < T0.size(); j++){
        out << QString::number(rob[i]->y_v[j]) << " ";
    } out << "\n";

    // Area of the box
    for(uint j = 0; j < T0.size(); j++){
        double length = Width(T0[j][2*i+1])*Width(T0[j][2*i+2]);
        out << QString::number(length) << " ";
    } out << "\n";

}

//--------------------------------------------------------------------------------------------------
void MainWindow::exportResults( const vector<box> &T ){

    QDir dir("./log");
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    for(uint i = 0; i < rob.size(); i++){
        logRobot(i);

    }
}



//--------------------------------------------------------------------------------------------------
// Run the intervals algorithm
void MainWindow::runLocalisation()
{

    int step = distances.size();
    T0 = vector<box>(step,box(interval(-oo,oo), 2*rob.size()));

    for(uint i = 0; i < rob.size(); i++){
        T0[0][2*i+1] = interval(rob[i]->x_v[0]) + interval(-0.05, 0.05);
        T0[0][2*i+2] = interval(rob[i]->y_v[0]) + interval(-0.05, 0.05);
    }

    T0[0][1] = interval(rob[0]->x_v[0]) + interval(-0.05, 0.05);
    T0[0][2] = interval(rob[0]->y_v[0]) + interval(-0.05, 0.05);

    T0[0][7] = interval(rob[3]->x_v[0]) + interval(-0.05, 0.05);
    T0[0][8] = interval(rob[3]->y_v[0]) + interval(-0.05, 0.05);



    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->N_outliers = ui->N_outliers->value();
    sivia->runAll2(T0,&rob,distances);

    outliers  = sivia->findOutliers(T0,&rob,distances);

    on_drawAllButton_clicked();
    checkIntegrity(T0);
    exportResults(T0);
    update_interface();
}

//--------------------------------------------------------------------------------------------------
// check if the true position of each robot belong the the corresponding box
void MainWindow::checkIntegrity(vector<box> &T0){
    for(int i = T0.size()-1; i >= 0; i--){
        for(uint j = 0; j < rob.size(); j++){
            Robot* r = rob[j];
//            qDebug() << T0[i].Width();
//            if(j == 2){
//                qDebug() << "X " << T0[i][2*j+1] << " Y " << T0[i][2*j+2];
//            }
            bool t = (T0[i][2*j+1].contains(r->x_v[i]) && T0[i][2*j+2].contains(r->y_v[i]));
            if(t == false){
                qDebug() << "error the true position isn't inside the box at step " << i << "and robot " << j;
                qDebug() << "X " << T0[i][2*j+1] << " Y " << T0[i][2*j+2];
                return;
            }
        }
    }
    qDebug() << "Integrity test passed";
}

//======================================================================================================
//================================= DRAWING FUNCTIONS ==================================================
//======================================================================================================
//======================================================================================================

// Draw all robots at time <step>
void MainWindow::drawRobots(int step){
    for(uint i = 0; i < rob.size(); i++){
        Rworld->DrawRobot(rob[i]->x_v[step],rob[i]->y_v[step],rob[i]->theta_v[step],0.2,color[i], brushs[i]);
    }
}

// Draw the trajectory of the robot number <robot_num>
void MainWindow::drawTrajectory(int robot_num){
    Robot* r = rob[robot_num];
    for(uint i = 1; i < r->x_v.size(); i++){
        Rworld->DrawLine(r->x_v[i-1], r->y_v[i-1],r->x_v[i], r->y_v[i],color[robot_num]);
        if( i%10 == 0){
            Rworld->DrawRobot(r->x_v[i],r->y_v[i],r->theta_v[i],0.1,color[robot_num], brushs[robot_num]);
        }
    }
}
// Draw trajectory of all robots
void MainWindow::drawAllTrajectories(){
    for(uint i = 0; i < rob.size(); i++){
        drawTrajectory(i);
    }
}

// Draw distances between the robot <num_rob> to all others at the time step <pos>
void MainWindow::drawCircles(int num_rob, int pos)
{
    //int num_rob = ui->selectRob->currentIndex();
    iMatrix dists = distances.at(pos);
    for(uint i = 0; i < rob.size(); i++){
        for(uint j = num_rob; j < num_rob+1/*rob.size()*/; j++){
            if(i == j) continue;
            Rworld->DrawEllipse(rob[i]->x_v[pos],rob[i]->y_v[pos],dists[j][i].inf,color[i],QBrush(Qt::NoBrush));
            Rworld->DrawEllipse(rob[i]->x_v[pos],rob[i]->y_v[pos],dists[j][i].sup,color[i],QBrush(Qt::NoBrush));
        }
    }
}

// draw boxes which enclosed true position of the robot at time step <step>
void MainWindow::drawBoxesState(int step){
    for(uint j = 0; j < rob.size(); j++){
        Rworld->DrawBox(T0[step][2*j+1].inf,T0[step][2*j+1].sup,T0[step][2*j+2].inf,T0[step][2*j+2].sup,QPen(Qt::black),QBrush(Qt::NoBrush));
    }
}


void MainWindow::drawOutliers(){
    int size = outliers.size()/3;
    for(uint i = 0; i < size; i++){
        int step = outliers[3*i];
        int idx = outliers[3*i+1];
        int idy = outliers[3*i+2];
        Rworld->DrawBox(T0[step][2*idx+1].inf,T0[step][2*idx+1].sup,T0[step][2*idx+2].inf,T0[step][2*idx+2].sup,QPen(Qt::red),QBrush(Qt::red));
        Rworld->DrawBox(T0[step][2*idy+1].inf,T0[step][2*idy+1].sup,T0[step][2*idy+2].inf,T0[step][2*idy+2].sup,QPen(Qt::green),QBrush(Qt::green));

//        Rworld->DrawLine(Center(T0[step][2*idx+1]),Center(T0[step][2*idx+2]),
//                Center(T0[step][2*idy+1]), Center(T0[step][2*idy+2]),QPen(Qt::red));
    }
}

//======================================================================================================
//===================================== GUI FUNCTIONS ==================================================
//======================================================================================================
//======================================================================================================


//--------------------------------------------------------------------------------------------------
void MainWindow::update_interface(){
    ui->resultBar->setMaximum(T0.size()-1);
    ui->resultBox->setMaximum(T0.size()-1);
    ui->resultBar->setValue(0);
    ui->resultBox->setValue(0);
}

//--------------------------------------------------------------------------------------------------
void MainWindow::keyPressEvent(QKeyEvent *event)
{   switch (event->key())
    {
    case Qt::Key_A: u1=u1+0.1;          break;
    case Qt::Key_E: u1=u1-0.1;          break;
    case Qt::Key_Z: u2=u2+0.1;          break;
    case Qt::Key_S: u2=u2-0.1;          break;
    }
}

//--------------------------------------------------------------------------------------------------
void MainWindow::on_N_outliers_editingFinished()
{
    sivia->N_outliers = ui->N_outliers->value();
}
//--------------------------------------------------------------------------------------------------
void MainWindow::on_EpsilonSpinBox_editingFinished()
{
    sivia->epsilon = ui->EpsilonSpinBox->value();
}
//--------------------------------------------------------------------------------------------------
void MainWindow::on_clearButtton_clicked()
{
    Rsivia->Clean();
    Rworld->Clean();
    drawRobots();
}

//--------------------------------------------------------------------------------------------------
void MainWindow::on_runTestBtn_clicked()
{
    generateData(ui->nbStep->value());

}
void MainWindow::on_resultBox_valueChanged(int arg1)
{
    ui->resultBar->setValue(arg1);
}

//--------------------------------------------------------------------------------------------------
void MainWindow::on_resultBar_valueChanged(int position)
{

    ui->resultBox->setValue(position);
    Rworld->Clean();
    drawRobots(position);
    drawBoxesState(position);
    if(ui->drawCircles->isChecked()){
        int indice = ui->selectRob->currentIndex();
        drawCircles(indice,position);
    }
}
//--------------------------------------------------------------------------------------------------


void MainWindow::on_drawAllButton_clicked()
{
    Rworld->Clean();
    drawRobots(0);
    drawAllTrajectories();
    for(uint i = 0; i < T0.size(); i++){
        drawBoxesState(i);
    }
    if(!T0[0].IsEmpty())
        drawOutliers();
}

//--------------------------------------------------------------------------------------------------
void MainWindow::on_contractAll_clicked()
{
    runLocalisation();
}
