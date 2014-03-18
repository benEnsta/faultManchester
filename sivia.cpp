#include "sivia.h"

// Global Variable
extern double dt;


SIVIA::SIVIA() : QObject()
{
    N_outliers = 0;
}

SIVIA::~SIVIA(){
}


void SIVIA::runAll(vector<box>& T0,vector<Robot*> *rob, vector<iMatrix> &distance){
    int ny = distance.size();

    double err[] = {0.000, 0.01, 0.01, 0.01,0.01, 0.01,0.01, 0.01};
    // Forward propagation
    for(int i = 1; i < ny; i++){ // i => indice du temps
        // X(k) = f(X(k-1))
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Incremente(T0[i][2*j+1],T0[i][2*j+2], T0[i-1][2*j+1], T0[i-1][2*j+2], r->theta_v[i-1],r->speed_v[i-1],err[j]);
        }
        // ******************************************************************************
        // HERE CALL THE CONTRACTOR TO FIND X_hat = G⁻1(Y) and make X = Inter(X, X_hat)
        // ******************************************************************************
        fixPoint(T0[i],distance[i]);
    }

    // Backward propagation
    for(int i = ny-1; i > 0; i--){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Decremente(T0[i][2*j+1],T0[i][2*j+2], T0[i-1][2*j+1], T0[i-1][2*j+2], r->theta_v[i-1],r->speed_v[i-1]);
        }
    }
}


void SIVIA::runAll2(vector<box>& T0,vector<Robot*> *rob, vector<iMatrix> &distance){


    int nb_step = T0.size();
    int step = rob->size()*2;
    vector<box> L;

    // Forward propagation
    box X = vector2box(T0);
    Ctrajectory(X,rob);

    for(uint i = 0; i < nb_step; i++){
        for(uint j = 0; j < rob->size(); j++){
            for(uint k = 0; k < rob->size(); k++){
                if(j == k) continue;
                interval i1(distance[i][j][k]);
                contractCircle(X[i*step + 2*j + 1],X[i*step + 2*j + 2],
                               X[i*step + 2*k + 1],X[i*step + 2*k + 2],i1);
                Ctrajectory(X,rob);
                L.push_back(X);
            }
        }
        qDebug()<< "step number :" << i;
    }
    C_q_in(X, L.size()-N_outliers, L);
    T0 = box2vector(X,2*rob->size());
}



void SIVIA::fixPoint(box &X, iMatrix &distance){
    bool sortie=false;
    while (!sortie)
    {  box Xold(X);
        // Call the contracteur
        outerContractAll2(X,distance);
        if (X.IsEmpty())      sortie=true;
        if (decrease(Xold,X)<0.005) sortie=true;
    }
}


void SIVIA::Ctrajectory(box &X,vector<Robot*> *rob){
    int nb_state = rob->at(0)->x_v.size()-1;
    int step = 2*rob->size();
    double err[] = {0.001, 0.01, 0.01, 0.01,0.01, 0.01,0.01, 0.01};
    for(int i = 1; i < nb_state; i++){ // i => indice du temps
        // X(k) = f(X(k-1))
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Incremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2],
                    X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2],
                    r->theta_v[i-1],r->speed_v[i-1],err[j]);
        }
    }

    // Backward propagation
    for(int i = nb_state-1; i > 0; i--){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Decremente(X[i*step + 2*j+1],X[i*step + 2*j+2],
                    X[(i-1)*step + 2*j+1], X[(i-1)*step + 2*j+2],
                    r->theta_v[i-1],r->speed_v[i-1]);
        }
    }
}

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

//---------------------------------------------------------------------------
void SIVIA::Incremente(box& X,box &X0, double theta, double vit, double err){
    for(uint i = 0; i < X0.dim*0.5; i++){
        Incremente(X[2*i+1], X[2*i+2], X0[2*i+1], X0[2*i+2],theta, vit, err);
    }
}

//---------------------------------------------------------------------------
void SIVIA::Incremente(interval& X1,interval& Y1, interval& X,interval& Y,
                       double theta,double vit, double noise)
{
    X1    = Inter(X1, X     + dt*cos(theta)*vit + interval(-noise, noise)) ;
         Y1    = Inter(Y1, Y     + dt*sin(theta)*vit + interval(-noise, noise)) ;
}
//---------------------------------------------------------------------------
void SIVIA::Decremente(box &X, box &X0, double theta, double vit, double err){
    for(int i = 0; i < X0.dim*0.5; i++){
        Decremente(X[2*i+1], X[2*i+2], X0[2*i+1], X0[2*i+2],theta, vit);
    }
}
//----------------------------------------------------------------------
void SIVIA::Decremente(interval& X1,interval& Y1, interval& X,interval& Y,
                       double theta,double V)
{
    double dx = dt*cos(theta)*V;
    double dy = dt*sin(theta)*V;
    Cmoins(X, X1,dx, -1);// - interval(-0.005, 0.005);
    Cmoins(Y, Y1,dy, -1);// - interval(-0.005, 0.005);
}


//-----------------------------------------------------------------
//-------------------  CONTRACTOR           -----------------------
//-----------------------------------------------------------------


//-----------------------------------------------------------------
//-------------------  CONTRACTOR ALL      -----------------------
//-----------------------------------------------------------------

void SIVIA::outerContractAll2(box& X, iMatrix &distances){

    vector<box> L;
    for(uint r = 1; r < 0.5*X.dim; r++){
        for(uint i = 0; i < 0.5*X.dim; i++){
            box X1(X);
            if(i == r) continue;
            interval i1(distances[r][i]);
            contractCircle(X1[2*r+1],X1[2*r+2],X1[2*i+1],X1[2*i+2],i1);
            if(X1.IsEmpty()) qDebug() << "empty result" << i << " " << r;
            L.push_back(X1);
        }
    }
    C_q_in(X, L.size()-N_outliers, L);
}

//-----------------------------------------------------------------
//-------------------  BASICS CONTRACTOR      ---------------------
//-----------------------------------------------------------------
void SIVIA::contractCircle(interval& x0,interval& y0, interval& x1, interval& y1, interval& d){


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
