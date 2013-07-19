#include "sivia.h"

// Global Variable


SIVIA::SIVIA() : QObject()
{
    C = ROB1;
}

SIVIA::~SIVIA(){
}

void SIVIA::doWork(box X0)
{
    Init();

    SIVIA_f(X0);
    emit workFinished();
}


void SIVIA::Init()
{

}

void SIVIA::getDistances(vector<robot*>& robs){
    for(int i = 0; i < robs.size(); i++){
        for(int j = 0; j < robs.size(); j++){
            if(robs[i] == robs[j]) continue;

            dist[i][j] = interval(robs[i]->getDistanceTo(robs[j]->x, robs[j]->y)) + interval(-0.1,0.1);
        }
    }
}

int SIVIA::SIVIA_f(box X0)    // Un contracteur est donné comme paramètre : on passe un pointeur sur fonction.
{   list<box> L;
    int k=0;
    box X(X0);
    L.push_back (X);
    while ( !L.empty() )
    {   k++;
        qDebug()<<"k="<<k<< "box width "<< X.Width();
        X=L.front();   L.pop_front();
        //emit drawBox(X,1);
        bool sortie=false;
        while (!sortie)
        {  box Xold(X);
            emit drawBox(X,1);
            // Contracteur
            switch(C){
                case ROB1: contractRX(X,1); break;
                case ROB2: contractRX(X,2); break;
                case ROB3: contractRX(X,3); break;
                case ROBALL: contractAll(X); break;
            }


            if (X.IsEmpty())      sortie=true;
            if (decrease(Xold,X)<0.05) sortie=true;
        }

        if (!X.IsEmpty())
        {  if (X.Width()<epsilon)
            {
                emit drawBox(X,2);
                qDebug() << X;
                //Rsivia->DrawBox(X[1].inf,X[1].sup,X[2].inf,X[2].sup,QPen(Qt::red),QBrush(Qt::red));
                //Rworld->DrawRobot(Center(X[1]),Center(X[2]),Center(X[3]),0.1);
            }
            else
            {   box X1(2);  box X2(2);
                Bisect (X,X1,X2);
                L.push_back(X1);  L.push_back(X2);
            }
        }
        //if (k > 1000) break;
    }

    qDebug() << "Number of boxes "<<k;
    //Rsivia->Save("paving");
}


void SIVIA::contractAll(box& X){
    vector<box> L;


    for(int i = 0; i < X.dim/2; i++){
        for(int j = 0; j < X.dim/2; j++){
            if (i == j) continue;
            box Xtmp(X);
            contractCircle(Xtmp[2*i+1],Xtmp[2*i+2], Xtmp[2*j+1], Xtmp[2*j+2],   dist[i][j]);
            L.push_back(Xtmp);
        }
    }

    C_q_in(X, L.size(), L);
}

void SIVIA::contractAll2(box& X){
    //vector<box> L;


    //box X1(X);
    contractRX(X,1);
    //L.push_back(X1);
    //box X2(X);
    contractRX(X,2);
    contractRX(X,3);
    //L.push_back(X2);
    //C_q_in(X, L.size(), L);
    //X[3] = X1[1];
    //X[4] = X1[2];
    //X[5] = X2[1];
    //X[6] = X2[2];

}

void SIVIA::contractR2(box& X){
    box b0 = box(X[5],X[6]);
    box b1 = box(X[5],X[6]);
    box b2 = box(X[5],X[6]);
    box b3 = box(X[5],X[6]);
    box b4 = box(X[5],X[6]);
    box b5 = box(X[5],X[6]);
    box b6 = box(X[5],X[6]);
    vector<box> L;


    contractCircle(X[1],X[2], b1[1], b1[2],   dist[0][2]);
    L.push_back(b1);

    contractCircle(b2[1], b2[2], X[1], X[2],  dist[2][0]);
    L.push_back(b2);

    contractCircle(b3[1], b3[2], X[3],X[4],   dist[1][2]);
    L.push_back(b3);

    //contractCircle(X[5],X[6], X[1],X[2],  dist[2][0]);
    contractCircle(X[3],X[4], b4[1], b4[2],   dist[2][1]);
    L.push_back(b4);

    contractCircle(X[7],X[8], b5[1], b5[2],   dist[3][2]);
    L.push_back(b5);

    contractCircle(X[7],X[8], b6[1], b6[2],   dist[2][3]);
    L.push_back(b6);


    C_q_in(b0, L.size(), L);
    //b0 = Inter(b2, b3);
    //b0 = b2;
    X[5] = b0[1];
    X[6] = b0[2];


}

void SIVIA::contractR1(box& X){
    box b0 = box(X[3],X[4]);
    box b1 = box(X[3],X[4]);
    box b2 = box(X[3],X[4]);
    box b3 = box(X[3],X[4]);
    box b4 = box(X[3],X[4]);
    box b5 = box(X[3],X[4]);
    vector<box> L;


    contractCircle(X[1],X[2], b1[1], b1[2],  dist[0][1]);
    L.push_back(b1);

    contractCircle(b2[1], b2[2], X[1], X[2],  dist[1][0]);
    L.push_back(b2);

    contractCircle(b3[1], b3[2], X[5],X[6],  dist[1][2]);
    L.push_back(b3);

    //contractCircle(X[5],X[6], X[1],X[2],  dist[2][0]);
    contractCircle(X[5],X[6], b4[1], b4[2],  dist[2][1]);
    L.push_back(b4);

    contractCircle(X[7],X[8], b5[1], b5[2],  dist[3][1]);
    L.push_back(b5);

    C_q_in(b0, L.size(), L);
    //b0 = Inter(b2, b3);
    //b0 = b2;
    X[3] = b0[1];
    X[4] = b0[2];


}

void SIVIA::contractRX(box& X, int r){
    vector<box> L;
    for(int i = 0; i < X.dim/2; i++){
        if(i == r) continue;
        box b1 = box(X[2*r+1],X[2*r+2]);
        box b2 = box(X[2*r+1],X[2*r+2]);
        contractCircle(X[2*i+1],X[2*i+2], b1[1], b1[2],  dist[i][r]);
        L.push_back(b1);

        //contractCircle(b2[1], b2[2], X[2*i+1], X[2*i+2],  dist[r][i]);
        //L.push_back(b2);
    }
    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, L.size(), L);
    X[2*r+1] = b0[1];
    X[2*r+2] = b0[2];



}



void SIVIA::contractCircle(interval& x0,interval& y0, interval& x1, interval& y1, interval& d){

//    interval x0 =R0[1];
//    interval y0 =R0[2];
//    interval x1 =R1[1];
//    interval y1 =R1[2];

    // (x0-x1)^2+(y0-y1)^2 = d^2
    interval dx=x0 - x1;
    interval dy=y0 - y1;
    interval dx2=Sqr(dx);
    interval dy2=Sqr(dy);
    interval dx2pdy2=dx2+dy2;
    interval r2=Sqr(d);
    Cegal(r2,dx2pdy2);


    //Csqr(r2,d,-1);
    Cplus(dx2pdy2,dx2,dy2,-1);
    Csqr(dy2,dy,-1);
    Csqr(dx2,dx,-1);
    Cmoins(dx,x0,x1,-1);
    Cmoins(dy,y0,y1,-1);
//    R0[1] = x0;
//    R0[2] = y0;
//    R1[1] = x1;
//    R1[2] = y1;
}


box SIVIA::getResult(){
    if(result.size() == 0) return box();
    box res = result[result.size()-1];
    for(int i= 0; i < result.size(); i++){
        res = Inter(res,result[i]);
    }
    qDebug()<< result[result.size()-1];
    return res;
}
