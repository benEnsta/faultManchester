#include "sivia.h"

// Global Variable
extern double dt;

SIVIA::SIVIA() : QObject()
{

    C = ROB1;
    m = 3;
    N_outliers = 2;
    reccordNumber = 0;
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
    if(X1.IsEmpty()){
        res->push_back(X0);
        return res;
    }
    interval cx1, cx2, cy1, cy2;
    diffI(X0[1],X1[1],cx1, cx2);
    diffI(X0[2],X1[2],cy1, cy2);

    if(cx1.inf != cx1.sup && !cx1.isEmpty && !X0[2].isEmpty) res->push_back(box(cx1,interval(X0[2])));
    if(cx2.inf != cx2.sup && !cx2.isEmpty && !X0[2].isEmpty) res->push_back(box(cx2,interval(X0[2])));

    if(cx1.isEmpty && cx2.isEmpty){
        if(cy1.inf != cy1.sup && !cy1.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1]),cy1));
        if(cy2.inf != cy2.sup && !cy2.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1]),cy2));
    } else if(!cx1.isEmpty && !cx2.isEmpty){
        if(cy1.inf != cy1.sup && !cy1.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(cx1.sup,cx2.inf),cy1));
        if(cy2.inf != cy2.sup && !cy2.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(cx1.sup,cx2.inf),cy2));
    } else if(!cx1.isEmpty && cx2.isEmpty){
        if(cy1.inf != cy1.sup && !cy1.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(cx1.sup,X0[1].sup),cy1));
        if(cy2.inf != cy2.sup && !cy2.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(cx1.sup,X0[1].sup),cy2));
    } else if(cx1.isEmpty && !cx2.isEmpty) {
        if(cy1.inf != cy1.sup && !cy1.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1].inf,cx2.inf),cy1));
        if(cy2.inf != cy2.sup && !cy2.isEmpty && !X0[1].isEmpty) res->push_back(box(interval(X0[1].inf,cx2.inf),cy2));
    }else{
        qDebug("cas non repertorier");
    }
    return res;
}
//----------------------------------------------------------------------------
void SIVIA::doWork(box X0, vector< vector< interval> > *dist)
{
    Init();
    result.clear();
    this->dist = *dist;
    SIVIA_f(X0);
    emit workFinished();
}


void SIVIA::doContractState(box X0, vector<robot*> *rob, vector<vector<vector<interval> > > *distance)
{
    Init();
    this->rob = rob;
    this->distance = distance;
    C = STATE;
    contractState(X0);
    //SIVIA_f(X0);
    emit workFinished();
}


void SIVIA::doStepR1(box X0, vector<robot*> *rob, vector<vector<vector<interval> > > *distance){
    this->distance = distance;
    this->rob = rob;
    C = STEPR1;
    SIVIA_f(X0);
    emit workFinished();
}

void SIVIA::Init()
{

}

void SIVIA::getDistances(vector<robot*>& robs){
    dist.resize(rob->size(), vector < interval> (rob->size()));
    for(uint i = 0; i < robs.size(); i++){
        for(uint j = 0; j < robs.size(); j++){
            if(i == j) continue;
            dist[i][j] = interval(robs[i]->getDistanceTo(robs[j]->x, robs[j]->y)) + interval(-0.05,0.05);
        }
    }
}

void SIVIA::stateEstim(box& X,vector<robot*> &robs){
    for(uint i = 0; i < robs.size(); i++){
        X[2*i+1] = X[2*i+1] + dt*robs[i]->vit*cos(robs[i]->theta) + interval(-0.001, 0.001);
        X[2*i+2] = X[2*i+2] + dt*robs[i]->vit*sin(robs[i]->theta) + interval(-0.001, 0.001);
    }
    //emit drawBox(X,1);
}

//---------------------------------------------------------------------------
void SIVIA::Incremente(box& X0,box X, double theta, double vit){
    for(uint i = 0; i < X0.dim*0.5; i++){
        Incremente(X[2*i+1], X[2*i+2], X0[2*i+1], X0[2*i+2],theta, vit, 0.001);
    }
}

//---------------------------------------------------------------------------
void SIVIA::Incremente(interval& X1,interval& Y1, interval& X,interval& Y,
                        double theta,double vit, double noise)
{        X1    = Inter(X1, X     + dt*cos(theta)*vit + interval(-noise, noise)) ;
         Y1    = Inter(Y1, Y     + dt*sin(theta)*vit + interval(-noise, noise)) ;
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
{
    double dx = dt*cos(theta)*V;
    double dy = dt*sin(theta)*V;
    Cmoins(X, X1,dx, -1);// - interval(-0.005, 0.005);
    Cmoins(Y, Y1,dy, -1);// - interval(-0.005, 0.005);
}
//----------------------------------------------------------------------


int SIVIA::SIVIA_f(box X0)
{   list<box> L;
    int ind= 0;
    int k=0;
    box X(X0);
    L.push_back (X);
    while ( !L.empty() )
    {   k++;
        qDebug()<<"k="<<k<< "box width "<< X.Width();// <<"Axe princ " << AxePrincipal(X) ;
        X=L.front();   L.pop_front();
        //emit drawBox(X,1);
        bool sortie=false;
        while (!sortie)
        {  box Xold(X);
            //emit drawBox(X,1);
            // Contracteur
            switch(C){
                case ROB1: contract_and_draw(X,1); break;
                case ROB2: contract_and_draw(X,2); break;
                case ROB3: contract_and_draw(X,3); break;
                case ROBALL: contractInOut(X); break;
                case STATE: contractState(X); break;
                case STEPR1: contractReccord(X);
            }


            if (X.IsEmpty())      sortie=true;
            if (decrease(Xold,X)<0.05) sortie=true;
        }

        if (!X.IsEmpty())
        {  if (X.Width()<epsilon)
            {
                emit drawBox(box(X[3],X[4]),6);
                result.push_back(X);
                ind++;
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
    emit drawBox(X,7);
    //Rsivia->Save("paving");
    return 0;
}


void SIVIA::contractState(box& X){

    int ny = distance->size();
    int step = 2*rob->size();
    double err[] = {0.000, 0.01, 0.01, 0.01};
    // Forward propagation
    for(int i = 1; i < ny; i++){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice robot
            robot* r = rob->at(j);
            Incremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2], X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2], r->theta_v[i],r->speed_v[i],err[j]);
        }

        box tmp0 = X.extract(i*step, 2*rob->size());
        incr.push_back(tmp0);
        box tmp;
        if(tmp0.Width() < 0.05) {
            tmp = Inflate(tmp0,0.1);
            tmp[1]  = tmp0[1];
            tmp[2]  = tmp0[2];
        } else
            tmp = tmp0;

        tmp[1]  = tmp0[1];
        tmp[2]  = tmp0[2];
        //qDebug() << "avant" << tmp[3] << tmp[4];
        //contractAt(tmp,distance->at(i));


        C = ROBALL;
        doWork(tmp,&distance->at(i));
        box Xr = getResult();
        cont.push_back(Xr);
        if(Xr.IsEmpty()) {
            qDebug("outliers possible %d", i);
            emit drawCircle(i);
            //break;
        } else {
            //tmp = Inter(tmp0,Xr);
        }
        emit drawBox(tmp,2);
        X.remplace(tmp,i*step);


    }
    C = STATE;

    // Backward propagation
    for(int i = ny-1; i > 0; i--){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice robot
            robot* r = rob->at(j);
            Decremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2], X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2], r->theta_v[i],r->speed_v[i]);
        }
    }


}

void SIVIA::contractAt(box& X, vector < vector < interval > > &dists){

    dist = dists;
    for(uint i = 1; i < X.dim*0.5; i++){
        outerContract(X,i);
    }
//    vector<box> L;
//    for(int i = 0; i < X.dim/2; i++){
//        for(int j = 1; j < X.dim*0.5; j++){
//            if (i == j) continue;
//            box Xtmp(X);
//            contractCircle(Xtmp[2*i+1],Xtmp[2*i+2], Xtmp[2*j+1], Xtmp[2*j+2],   dists[i][j]);
//            L.push_back(Xtmp);
//        }
//    }
//    for(int i = 0; i < L.size(); i++){
//        qDebug() << L[i][4];
//    }
//    C_q_in(X, L.size()-N_outliers, L);
    //emit drawBox(X,2);
    //qDebug("contrct");
//    for(int i = 0; i < dists.at(1).size(); i++){
//        for(int j = 0; j < dists.at(1).size(); j++){
//            dist[i][j] = dists[i][j];
//        }
//    }

//    contract_and_draw(X,1);
    //innerContract(X);
}



void SIVIA::contractInOut(box &X){
    for(uint i = 1; i < X.dim*0.5; i++){
        contract_and_draw(X,i);
    }
}

void SIVIA::contractInOut(box &X, vector< vector< interval> > distance){
    for(uint i = 1; i < rob->size(); i++){
        for(uint j = 0; j < rob->size(); j++){
            if(i == j) continue;
            dist[i][j] = distance[i][j];
        }
    }
    for(uint i = 1; i < X.dim*0.5; i++){
        contract_and_draw(X,i);
    }
}

void SIVIA::contract_and_draw(box &X, int r){
    vector<box>* dif;
    box Xold2(X);
    outerContract(X,r);
    dif = diff(box(Xold2[2*r+1],Xold2[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(int i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
         emit drawBox(Xt,4);
    }
    delete dif;
    if(X.IsEmpty())return ;
    box Xold(X);
    innerContract(X,r);
    dif = diff(box(Xold[2*r+1],Xold[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(uint i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
        emit drawBox(Xt,3);
    }
    delete dif;

}


void SIVIA::contractReccord(box &X){
    dist = distance->at(reccordNumber);
    qDebug("Contract");
    for(uint i = 0; i < X.dim*0.5; i++){
        contract_and_draw(X,i);
    }
}

void SIVIA::innerContract(box &X, int r){

    vector<box> L;
    for(uint i = 0; i < X.dim*0.5; i++){
        if (i == r) continue;
        box C00(box(X[2*i+1],X[2*i+2]));
        box X1(box(X[2*r+1],X[2*r+2]));
        interval i1(dist[r][i]);
        CdiskExists(X1[1],X1[2],C00[1],C00[2],i1,true);
        L.push_back(X1);
    }
    //C_q_in(X,N_outliers +1 ,L);

//    vector<box> L;
//    for(int i = 0; i < X.dim*0.5; i++){
//            if (i == r) continue;
//            box X1(X[2*r+1],X[2*r+2]), X2(X1);
//            box X01(X), X02(X);
//            interval dmoins(0,dist[i][r].inf);
//            interval d(dist[i][r]);
//            interval dplus(dist[i][r].sup, +oo);
//            CdiskExists(X1[1], X1[2], X01[2*i+1], X01[2*i+2],  d, true);
//            //contractCircle(X1[1], X1[2], Center(X01[2*i+1]),Center(X01[2*i+2]),   dmoins);
//            //contractCircle(X2[1], X2[2], Center(X02[2*i+1]),Center(X02[2*i+2]),    dplus);
//            L.push_back(X1);
//    }
    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, N_outliers + 1 , L);
    //b0 = Union(L);
    X[2*r+1] = b0[1];
    X[2*r+2] = b0[2];
}

void SIVIA::outerContract(box& X, int r){
    vector<box> L;

    for(uint i = 0; i < 0.5*X.dim; i++){
        if(i == r) continue;
        box C00(box(X[2*i+1],X[2*i+2]));
        box X1(box(X[2*r+1],X[2*r+2]));
        interval i1(dist[r][i]);
        CdiskExists(X1[1],X1[2],C00[1],C00[2],i1,false);
        L.push_back(X1);
    }

//    for(int i = 0; i < X.dim*0.5; i++){
//        if(i == r) continue;
//        box b1(X[2*r+1],X[2*r+2]);
//        //contractCircle(b1[1], b1[2], Center(X[2*i+1]), Center(X[2*i+2]),  dist[i][r]);
//        CdiskExists(b1[1], b1[2], X[2*i+1], X[2*i+2],  dist[i][r], false);
//        L.push_back(b1);
//    }
    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, L.size()-N_outliers, L);

    X[2*r+1] = b0[1];
    X[2*r+2] = b0[2];
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

box SIVIA::getResult(){
    if(result.size() == 0) return box();
    box res = result[result.size()-1];
    for(int i= 0; i < result.size(); i++){
        for(int j = 1; j <= 8 ; j++){
            box b = result.at(i);
            if(b[j].inf == -999){
                qDebug() << "empty restult "<< i <<" " << result[i] << "----------------";
                break;
            }
        }

        res = Union(res,result[i]);
    }
    qDebug()<< res;//result[result.size()-1];
    drawBox(res,0);
    return res;
}
