/**
 * \file      sivia.cpp
 * \author    Benoit DESROCHERS <benoitdesrochers@ensta-bretagne.org>
 * \version   1.0
 * \date      mar 24, 2014
 * \brief     SIVIA class which implements the localisation and fault detection algorithm
 */
#include "sivia.h"



// Global Variable
extern double dt;



SIVIA::SIVIA() : QObject()
{
    N_outliers = 0;
}
//---------------------------------------------------------------------------------------------
SIVIA::~SIVIA(){
}

//---------------------------------------------------------------------------------------------
/**
 * @brief Main Contractor
 *
 * @param T0 : vector of boxes. each box contains has a dimention of n*nr
 * @param rob: pointor to the vector of robots used by the contractor CTrajectory
 * @return vector<box>  list of boxes of size <boxSize>
 */

void SIVIA::runAll2(vector<box>& T0,vector<Robot*> *rob, vector<iMatrix> &distance){

    int nb_step = T0.size();
    int step = rob->size()*2;
    bool sortie=false;
    box X0 = vector2box(T0);
    while (!sortie)
    {  box Xold(X0);
        // Call the contracteur
        vector<box> L;
        // Forward propagation
        Ctrajectory(X0,rob);
        for(uint i = 0; i < nb_step; i++){
            for(uint j = 0; j < rob->size(); j++){
                for(uint k = 0; k < rob->size(); k++){
                    if(j == k) continue;
                    box X(X0);
                    interval i1(distance[i][j][k]);
                    contractCircle(X[i*step + 2*j + 1],X[i*step + 2*j + 2],
                                   X[i*step + 2*k + 1],X[i*step + 2*k + 2],i1);
                    Ctrajectory(X,rob);
                    if(X.IsEmpty()) X.setEmpty();
                    L.push_back(X);
                }
            }
            qDebug()<< "step number :" << i;
        }
        C_q_in(X0, L.size()-N_outliers, L);
        if (X0.IsEmpty())      sortie=true;
        if (decrease(Xold,X0)<0.1e-7) sortie=true;
    }


    if(X0.IsEmpty()){
        qDebug() << "X is Empty";
    }
    T0 = box2vector(X0,2*rob->size());
}



//---------------------------------------------------------------------------------------------
// This function identifies outliers.
// For each measurment we apply the contractor <contractCircle> with an initial box which contains the true position of the robot.
// if the resulting box is empty, the measurment is faulty. (cf theorem 3)
vector<int> SIVIA::findOutliers(vector<box>& T0,vector<Robot*> *rob, vector<iMatrix> &distance){
    int nb_step = T0.size();        // total number of time step
    int step = rob->size()*2;       //
    box X0 = vector2box(T0);

    vector<int> out;
    for(uint i = 0; i < nb_step; i++){
        for(uint j = 0; j < rob->size(); j++){
            for(uint k = 0; k < rob->size(); k++){
                if(j == k) continue;
                box X(X0);
                interval i1(distance[i][j][k]);
                contractCircle(X[i*step + 2*j + 1],X[i*step + 2*j + 2],
                               X[i*step + 2*k + 1],X[i*step + 2*k + 2],i1);
                Ctrajectory(X,rob);
                if(X.IsEmpty()){
                    qDebug() << "outliers " << i << " "<< j << " "<< k << " ";
                    out.push_back(i); out.push_back(j); out.push_back(k);
                }
            }
        }
    }

    return out;
}


//---------------------------------------------------------------------------------------------
// Forward / Backward contractor for the whole trajectory of each robot

void SIVIA::Ctrajectory(box &X,vector<Robot*> *rob){
    int nb_state = rob->at(0)->x_v.size()-1;
    int step = 2*rob->size();

    // Forward propagation
    for(int i = 1; i < nb_state; i++){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Incremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2],
                    X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2],
                    r->theta_v[i-1],r->speed_v[i-1],r->noise);
        }
    }

    // Backward propagation
    for(int i = nb_state-1; i > 0; i--){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Decremente(X[i*step + 2*j+1],X[i*step + 2*j+2],
                    X[(i-1)*step + 2*j+1], X[(i-1)*step + 2*j+2],
                    r->theta_v[i-1],r->speed_v[i-1],r->noise);
        }
    }
}


//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
//--------------------------    BASIC CONTRACTORS  --------------------------------------------
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
// Compute X1 = f(X) => forward
void SIVIA::Incremente(interval& X1,interval& Y1, interval& X,interval& Y,
                       double theta,double vit, double noise)
{
    X1    = Inter(X1, X     + dt*cos(theta)*vit + interval(-noise, noise)) ;
         Y1    = Inter(Y1, Y     + dt*sin(theta)*vit + interval(-noise, noise)) ;
}

//---------------------------------------------------------------------------------------------
// compute X = f^-1(X1) => backward
void SIVIA::Decremente(interval& X1,interval& Y1, interval& X,interval& Y,
                       double theta,double V, double err)
{
    interval dx = dt*cos(theta)*V;// + interval(-err,err);
    interval dy = dt*sin(theta)*V;// + interval(-err,err);
    Cplus(X1, dx, X, -1);// - interval(-0.005, 0.005);
    Cplus(Y1, dy, Y, -1);// - interval(-0.005, 0.005);

}


//---------------------------------------------------------------------------------------------
void SIVIA::contractCircle(interval& x0,interval& y0, interval& x1, interval& y1, interval& d){

    // Contractor : (x0-x1)^2+(y0-y1)^2 = d^2
    interval dx=x0 - x1;
    interval dy=y0 - y1;
    interval dx2=Sqr(dx);
    interval dy2=Sqr(dy);
    interval dx2pdy2=dx2+dy2;
    interval r2=Sqr(d);

    dx2pdy2 = r2 & dx2pdy2;

    Cplus(dx2pdy2,dx2,dy2,-1);
    Csqr(dy2,dy,-1);
    Csqr(dx2,dx,-1);
    Cmoins(dx,x0,x1,-1);
    Cmoins(dy,y0,y1,-1);

}

//---------------------------------------------------------------------------------------------
void SIVIA::contractCircle(interval& x0,interval& y0, double x1, double y1, interval& d){

    //qDebug() << "version avec centre double";

    // Contractor : (x0-x1)^2+(y0-y1)^2 = d^2
    interval dx=x0 - x1;
    interval dy=y0 - y1;
    interval dx2=Sqr(dx);
    interval dy2=Sqr(dy);
    interval dx2pdy2=dx2+dy2;
    interval r2=Sqr(d);

    Cegal(r2,dx2pdy2);

    Cplus(dx2pdy2,dx2,dy2,-1);
    Csqr(dy2,dy,-1);
    Csqr(dx2,dx,-1);
    Cmoins(dx,x0,x1,-1);
    Cmoins(dy,y0,y1,-1);

}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
//--------------------------    TOOL FUNCTIONS     --------------------------------------------
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

/**
 * @brief this function splits a <boxSize>*r dimentionnal box into a list of r boxes of size <boxSize>
 *
 * @param X : box to be transformed
 * @param boxSize number of dimention of the new box
 * @return vector<box>  list of boxes of size <boxSize>
 */
vector<box> SIVIA::box2vector(box X, int boxSize){
    vector<box> list(X.dim/(boxSize));
    for(uint i = 0; i < X.dim/boxSize; i++){
        box Xtmp(boxSize);
        for(uint j = 1; j <= boxSize; j++){
            Xtmp[j] = X[boxSize*i+j];
        }
        list[i] = Xtmp;
    }
    return list;
}

//----------------------------------------------------------------------------------------------
/**
 * @brief this function merges a list of r boxes of size 2 into a box of size 2*r
 *
 * @param X : box to be transformed
 * @return vector<box>  list with boxes
 */
box SIVIA::vector2box(vector<box> & list){
    if(list.size() > 0){
        int dim = list[0].dim;
        box X(list.size()*dim);
        for(uint i = 0; i < list.size(); i++){
            for(uint j = 1; j <= dim; j++)
            X[i*dim+j] = list[i][j];

        }
        return X;
    } else {
        return box();
    }
}


