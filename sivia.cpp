#include "sivia.h"

// Global Variable
extern double dt;

SIVIA::SIVIA() : QObject()
{

    C = ROB1;
    m = 3;
    N_outliers = 0;
}

SIVIA::~SIVIA(){
}

//----------------------------------------------------------------------
void diffI(interval &x0, interval &x1, interval &c0, interval &c1){
    interval xt = Inter(Inter(x0, x1)-x1,interval(0,0));
    if(x1.isEmpty || xt.isEmpty){
        c0 = interval(); c1 = interval();
    } else {
        c0 = (x1.inf == x0.inf) ? interval() :  interval(x1.inf,x0.inf);
        c1 = (x0.sup == x1.sup) ? interval() :  interval(x0.sup,x1.sup);
    }

}

vector<box> *diff(box X0, box X1){
    vector<box> *res = new vector<box>();
    if(X0.IsEmpty()) return res;
    interval cx1, cx2, cy1, cy2;
    diffI(X0[1],X1[1],cx1, cx2);
    diffI(X0[2],X1[2],cy1, cy2);

    if(!cx1.isEmpty && !X0[2].isEmpty) res->push_back(box(cx1,interval(X0[2])));
    if(!cx2.isEmpty && !X0[2].isEmpty) res->push_back(box(cx2,interval(X0[2])));
    if(!cy1.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1]),cy1));
    if(!cy2.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1]),cy2));
    return res;
}
//----------------------------------------------------------------------------
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

void SIVIA::stateEstim(box& X,vector<robot*> &robs){
    for(int i = 0; i < robs.size(); i++){
        X[2*i+1] = X[2*i+1] + dt*robs[i]->vit*cos(robs[i]->theta) + interval(-0.001, 0.001);
        X[2*i+2] = X[2*i+2] + dt*robs[i]->vit*sin(robs[i]->theta) + interval(-0.001, 0.001);
    }
    //emit drawBox(X,1);
}

//---------------------------------------------------------------------------
void SIVIA::Incremente(box& X0,box X, double theta, double vit){
    for(int i = 0; i < X0.dim*0.5; i++){
        Incremente(X[2*i+1], X[2*i+2], X0[2*i+1], X0[2*i+2],theta, vit);
    }
}

//---------------------------------------------------------------------------
void SIVIA::Incremente(interval& X1,interval& Y1, interval& X,interval& Y,
                        double theta,double vit)
{        X1    = X     + dt*cos(theta)*vit + interval(-0.001, 0.001);
         Y1    = Y     + dt*sin(theta)*vit + interval(-0.001, 0.001);
}
//---------------------------------------------------------------------------
void SIVIA::Decremente(box& X0,box X, double theta, double vit){
    for(int i = 0; i < X0.dim*0.5; i++){
        Decremente(X[2*i+1], X[2*i+2], X0[2*i+1], X0[2*i+2],theta, vit);
    }
}
//----------------------------------------------------------------------
void SIVIA::Decremente(interval& X1,interval& Y1, interval& X,interval& Y,
                double theta,double V)
{        X     = X1-dt*cos(theta)*V;
         Y     = Y1-dt*sin(theta)*V;
}
//----------------------------------------------------------------------

//bool SIVIA::Contract(box& P)
   /*vector<box> L;
    vector<box> Pi(m);
    bool deb=false;
    Pi[m-1]=P;
                //double mx=Form1->mx;
                //double my=Form1->my;

    vector<double>   Vu1(m);    for(int i=0;i<m;i++) {Vu1[i]=  0    };
    vector<double>   Vu2(m);    for(int i=0;i<m;i++) {Vu2[i]=  0    };
    //vector<double>   Valpha(m); list<double>::iterator iLalpha=Lalpha.begin(); for(int i=0;i<m;i++) {Valpha[i]=*iLalpha; iLalpha++;}
    vector<interval> Valpha(m); list <interval>::iterator iLalpha=Lalpha.begin();      for(int i=0;i<m;i++) {Valpha[i]=*iLalpha;iLalpha++;}

    for  (int i=m-2;i>=0;i--)
        {       Pi[i]=box(4);
                Decremente(Pi[i+1][1],Pi[i+1][2],Pi[i+1][3],Pi[i+1][4],Pi[i][1],Pi[i][2],Pi[i][3],Pi[i][4],Vu1[i],Vu2[i]);
        }   ;

    for  (int i=0;i<m;i++)
        {       interval xi    =Pi[i][1];   interval yi=Pi[i][2];
                interval thetai=Pi[i][3];   interval vi=Pi[i][4];
                interval betai=Valpha[i]+thetai;
                interval Sini=Sin(betai);
                interval Cosi=Cos(betai);
                interval w1=my*Cosi-mx*Sini;
                //Cosi*x2-Sini*x1=w1   i.e.  Cosi*yi-Sini*xi=w1
                xi=Inter(xi,(Cosi*yi-w1)/Sini);
                yi=Inter(yi,(w1+Sini*xi)/Cosi);
                Cosi=Inter(Cosi,(w1+Sini*xi)/yi);
                Sini=Inter(Sini,(Cosi*yi-w1)/xi);
                Csin(Sini,betai,-1);
                Ccos(Cosi,betai,-1);
                Cplus(betai,Valpha[i],thetai,-1);

                interval xip,yip,thetaip,vip;
                for  (int i1=i;i1<m-1;i1++)
                {     Incremente(xip,yip,thetaip,vip,xi,yi,thetai,vi,Vu1[i1],Vu2[i1]);
                      xip=Inter(xip,Pi[i1+1][1]);yip=Inter(yip,Pi[i1+1][2]);
                      thetaip=Inter(thetaip,Pi[i1+1][3]);vip=Inter(vip,Pi[i1+1][4]);
                      xi=xip; yi=yip; thetai=thetaip; vi=vip;
                };
                box Pb(P);
                Pb[1]=xi; Pb[2]=yi; Pb[3]=thetai; Pb[4]=vi;
                L.push_back(Pb);
         }
     vector<box> Lv=L;
     C_q_in(P,m-q,Lv);*/
//}

int SIVIA::SIVIA_f(box X0)
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
                result.push_back(X);
                //qDebug() << X;
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
    C_q_in(X, L.size()-N_outliers, L);
    innerContract(X);
}


void SIVIA::contract_and_draw(box &X, int r){
    vector<box>* dif;
    box Xold2(X);
    Outercontract(X,r);
    dif = diff(box(Xold2[2*r+1],Xold2[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(int i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
         emit drawBox(Xt,4);
    }
    delete dif;

    box Xold(X);
    Innercontract(X,r);
    dif = diff(box(Xold[2*r+1],Xold[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(int i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
        emit drawBox(Xt,4);
    }
    delete dif;

}


void SIVIA::innerContract(box &X){
    vector<box> L;
    for(int i = 0; i < X.dim/2; i++){
        for(int j = 0; j < X.dim/2; j++){
            if (i == j) continue;
            box X1(X), X2(X);
            interval dmoins(0,dist[i][j].inf);
            interval dplus(dist[i][j].sup, +oo);
            contractCircle(X1[2*i+1],X1[2*i+2], X1[2*j+1], X1[2*j+2],   dmoins);
            contractCircle(X2[2*i+1],X2[2*i+2], X2[2*j+1], X2[2*j+2],   dplus);
            L.push_back(Union(X1,X2));
        }
    }
    C_q_in(X, N_outliers + 1 , L);
}

void SIVIA::contractAll2(box& X){
    //vector<box> L;


    box X1(X);
    for(int i = 1; i < X.dim/2; i++){
        contractRX(X,i);
    }

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
    for(int i = 0; i < X.dim*0.5; i++){
        if(i == r) continue;
        box b1 = box(X[2*r+1],X[2*r+2]);
        contractCircle(X[2*i+1],X[2*i+2], b1[1], b1[2],  dist[i][r]);
        L.push_back(b1);
    }
    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, L.size()-N_outliers, L);

    X[2*r+1] = b0[1];
    X[2*r+2] = b0[2];
    L.clear();
    for(int i = 0; i < 3/*X.dim/2*/; i++){
        if (i == r) continue;
        box X1(X[2*r+1],X[2*r+2]) , X2(X[2*r+1],X[2*r+2]);
        interval dmoins(0,dist[i][r].inf);
        interval dplus(dist[i][r].sup, +oo);
        contractCircle(X[2*i+1],X[2*i+2], X1[1], X1[2],   dmoins);
        contractCircle(X[2*i+1],X[2*i+2], X2[1], X2[2],   dplus);
        L.push_back(Union(X1,X2));
    }
    box bIn = box(X[2*r+1],X[2*r+2]);
    C_q_in(bIn, N_outliers + 1 , L);
    X[2*r+1] = bIn[1];
    X[2*r+2] = bIn[2];

}



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


box SIVIA::getResult(){
    if(result.size() == 0) return box();
    box res = result[result.size()-1];
    for(int i= 0; i < result.size(); i++){
        res = Union(res,result[i]);
    }
    qDebug()<< result[result.size()-1];
    drawBox(res,0);
    return res;
}
