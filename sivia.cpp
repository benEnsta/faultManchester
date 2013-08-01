#include "sivia.h"

// Global Variable
extern double dt;
vector<box> *diff(box X0, box X1);
void diffI(interval &x0, interval &x1, interval &c0, interval &c1);


SIVIA::SIVIA() : QObject()
{

    C = ROB1;
    m = 3;
    N_outliers = 0;
    reccordNumber = 0;
    contractor = &SIVIA::contract_and_draw;
}

SIVIA::~SIVIA(){
}


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


void SIVIA::doContractOneByOne(box &X, vector< vector< interval> > *dist){
    Init();
    result.clear();
    P0.clear();
    this->dist = *dist;
    // Copy initial position.
    for(uint i = 0; i < X.dim*0.5; i++){
        P0.push_back(box(X[2*i+1],X[2*i+2]));
    }

    currentRob = 1;
    // DO sivia for each Robot.
    //for(uint j = 0; j < 10; j++){
        currentRob = 1;
        for(uint i = 1; i < P0.size(); i++){
            C = ROBUNIT;
            result.clear();
            box X0(X[2*i+1],X[2*i+2]);
            SIVIA_f(X0);
            box Xr = getResult();
            emit drawBox(Xr, 8);
            //P0[i] = Inter(P0[i],Xr);
            emit drawBox(P0[i], 6);
            currentRob++;
        }
   // }


}

void SIVIA::Init()
{

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
        qDebug()<<"k="<<k<< "box width "<< X.Width() <<"Axe princ " << AxePrincipal(X) ;
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
            case ROBUNIT: (this->*contractor)(X,0); break;
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
                emit drawBox(box(X[1],X[2]),6);
                result.push_back(X);
                ind++;
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
//------------------------------------------------------------------------
void SIVIA::contractInOut(box &X){
    for(uint i = 1; i < X.dim*0.5; i++){
        contract_and_draw(X,i);
    }
}

void SIVIA::contractReccord(box &X){
    dist = distance->at(reccordNumber);
    qDebug("Contract");
    for(uint i = 0; i < X.dim*0.5; i++){
        contract_and_draw(X,i);
    }
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

void SIVIA::contract_and_draw(box &X, int r){
    vector<box>* dif;
    box Xold2(X);
    if(r == 0)
        outerContractOne(X);
    else
        outerContractAll(X);
    dif = diff(box(Xold2[2*r+1],Xold2[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(int i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
        //emit drawBox(Xt,4);
    }
    delete dif;
    if(X.IsEmpty())return ;
    box Xold(X);
    if(r == 0){
        innerContractOne(X);
        if (X.IsEmpty())  result.push_back(X);
    } else {
        innerContractAll(X);
    }
    dif = diff(box(Xold[2*r+1],Xold[2*r+2]) , box(X[2*r+1],X[2*r+2]));
    for(uint i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
        //emit drawBox(Xt,3);
    }
    delete dif;

}


//-----------------------------------------------------------------
//-------------------  CONTRACTOR           -----------------------
//-----------------------------------------------------------------
void SIVIA::innerContract(box &X, int r){

    vector<box> L;
    for(uint i = 0; i < X.dim*0.5; i++){
        if (i == r) continue;
        box C00(box(X[2*i+1],X[2*i+2]));
        box X1(box(X[2*r+1],X[2*r+2]));
        interval i1(dist[r][i]);
        CdiskExists(X1[1],X1[2],C00[1],C00[2],i1,true);
        if(X1.IsEmpty()) L.push_back(box(X[2*r+1],X[2*r+2]));
        else
            L.push_back(X1);
    }

    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, N_outliers + 1 , L);

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

    box b0 = box(X[2*r+1],X[2*r+2]);
    C_q_in(b0, L.size()-N_outliers, L);

    X[2*r+1] = b0[1];
    X[2*r+2] = b0[2];
}

//-----------------------------------------------------------------
//-------------------  CONTRACTOR ALL      -----------------------
//-----------------------------------------------------------------

void SIVIA::outerContractAll(box& X){

    vector<box> L;
    for(uint r = 0; r < 0.5*X.dim; r++){
        for(uint i = 0; i < 0.5*X.dim; i++){
            if(i == r) continue;
            box X1(X);
            interval i1(dist[r][i]);
            CdiskExists(X1[2*r+1],X1[2*r+2],X1[2*i+1],X1[2*i+2],i1,false);
            L.push_back(X1);
        }
    }
    C_q_in(X, L.size()-N_outliers, L);
}

void SIVIA::innerContractAll(box& X){

    vector<box> L;
    for(uint r = 0; r < 0.5*X.dim; r++){
        for(uint i = 0; i < 0.5*X.dim; i++){
            if(i == r) continue;
            box X1(X);
            interval i1(dist[r][i]);
            CdiskExists(X1[2*r+1],X1[2*r+2],X1[2*i+1],X1[2*i+2],i1,true);
            L.push_back(X1);
        }
    }
    C_q_in(X, N_outliers +1, L);
}


//-----------------------------------------------------------------
//-------------------  CONTRACTOR ONE      -----------------------
//-----------------------------------------------------------------

void SIVIA::outerContractOne(box& X){

    vector<box> L;

    for(uint i = 0; i < P0.size(); i++){
        if(i == currentRob) continue;
        box X1(X);
        interval i1(dist[currentRob][i]);
        CdiskExists(X1[1],X1[2],P0[i][1],P0[i][2],i1,false);
        L.push_back(X1);
    }

    C_q_in(X, L.size()-N_outliers, L);
}

void SIVIA::innerContractOne(box& X){

    vector<box> L;
    for(uint i = 0; i < P0.size(); i++){
        if(i == currentRob) continue;
        box X1(X);
        interval i1(dist[currentRob][i]);
        CdiskExists(X1[1],X1[2],P0[i][1],P0[i][2],i1,true);
        L.push_back(X1);
    }
    C_q_in(X, N_outliers +1, L);
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


//---------------------------------------------------------------------
// return the union of all yellow boxes stored in the list result.
//---------------------------------------------------------------------
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
    //qDebug()<< res;//result[result.size()-1];
    //drawBox(res,0);
    return res;
}

//----------------------------------------------------------------------
//-----------------------------        TOOLS           -----------------
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
