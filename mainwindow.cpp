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
    rob.push_back(new Robot(8,7, M_PI_2));
    ui->selectRob->addItem("Robot 1");
    rob.push_back(new Robot(-8,4));
    ui->selectRob->addItem("Robot 2");
    rob.push_back(new Robot(-5,-9,-M_PI_2));
    ui->selectRob->addItem("Robot 3");
    rob.push_back(new Robot(-5,7));
//    ui->selectRob->addItem("Robot 4");
//    rob.push_back(new Robot(-4.5,-4.7));
//    ui->selectRob->addItem("Robot 5");
//    rob.push_back(new Robot(4.5,-3.5));
//    ui->selectRob->addItem("Robot 6");
    ui->selectRob->setCurrentIndex(1);

    ui->N_outliers->setMaximum(rob.size()-1);
    ui->resultBar->setMaximum(0);
    ui->resultBox->setMaximum(0);
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



// return the distance between the robot number n1 and n2 with a noise.
// Outlier generation could be added in this part
void MainWindow::generateDistances(int nb0){
    for(uint i = 0; i < nb0; i++){ // for each time step

        iMatrix dist(rob.size(),vector<interval> (rob.size()));
        for(uint k = 0; k < rob.size(); k++){ // for each pair of robot i,j
            for(uint j = 0; j < rob.size(); j++){
                if(k == j) continue;
                double trueDistance = hypot(rob[k]->x_v[i] - rob[j]->x_v[i], rob[k]->y_v[i] - rob[j]->y_v[i]);
                int noise_rd =  rand();
                double noise = -0.1 + 0.2*noise_rd/(RAND_MAX);
            //    qDebug() << noise_rd << "  " << -0.1 + 0.2*((double) noise_rd/(RAND_MAX));
                dist[k][j] = interval(trueDistance + noise) + interval(-0.1,0.1);
            }
        }
        distances.push_back(dist);
    }
}

//--------------------------------------------------------------------------------------------------
// Generate trajectories for all robot and all distances between them
void MainWindow::generateData(int nb0){

    // Clean previous datas
    distances.clear();
    // Generate robot's trajectory
    for(uint i = 0; i < rob.size(); i++){
        Robot* r = rob[i];
        r->generate8(2*i+1,nb0);
    }

    generateDistances(nb0);
    drawAllTrajectories();
}






//--------------------------------------------------------------------------------------------------
// Run the intervals algorithm
void MainWindow::runLocalisation()
{

    int step = distances.size();
    T0 = vector<box>(step,box(interval(-oo,oo), 2*rob.size()));

    for(uint i = 0; i < rob.size(); i++){
        T0[0][2*i+1] = interval(rob[i]->x_v[0]) + interval(-0.001, 0.001);
        T0[0][2*i+2] = interval(rob[i]->y_v[0]) + interval(-0.001, 0.001);
    }


    sivia->epsilon = ui->EpsilonSpinBox->value();
    sivia->N_outliers = ui->N_outliers->value();
    sivia->runAll2(T0,&rob,distances);




    on_drawAllButton_clicked();
    checkIntegrity(T0);
    update_interface();
}

//--------------------------------------------------------------------------------------------------
// check if the true position of each robot belong the the corresponding box
void MainWindow::checkIntegrity(vector<box> &T0){
    for(uint i = 0; i < T0.size(); i++){
        for(uint j = 0; j < rob.size(); j++){
            Robot* r = rob[j];
            bool t = (T0[i][2*j+1].contains(r->x_v[i]) && T0[i][2*j+2].contains(r->y_v[i]));
            if(t == false){
                qDebug() << "error the true position isn't inside the box at step " << i;
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
        Rworld->DrawBox(T0[step][2*j+1].inf,T0[step][2*j+1].sup,T0[step][2*j+2].inf,T0[step][2*j+2].sup,QPen(Qt::yellow),QBrush(Qt::NoBrush));
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
}

//--------------------------------------------------------------------------------------------------
void MainWindow::on_contractAll_clicked()
{
    runLocalisation();
}
