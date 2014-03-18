#include "sivia.h"

// Global Variable
extern double dt;
vector<box> *diff(box X0, box X1);
void diffI(interval &x0, interval &x1, interval &c0, interval &c1);


SIVIA::SIVIA() : QObject()
{
    m = 3;
    N_outliers = 2;
    reccordNumber = 0;
    contractor = &SIVIA::wrapperContractOneRobot;
}

SIVIA::~SIVIA(){
}


void SIVIA::doWork(box X0, int robotNumber, vector< vector< interval> > *dist)
{
    Init();
    // Clear Internal Data
    result.clear();
    P0.clear();
    this->dist = *dist;                                 // Set distances vector
    for(uint i = 0; i < X0.dim*0.5; i++){               // transform X0 into a list of boxes
        P0.push_back(box(X0[2*i+1],X0[2*i+2]));
    }
    contractor = &SIVIA::contract_and_drawOneRobot;       // set the contractor
    currentRob = robotNumber;                           // set who is the Robot to be computed
    SIVIA_f(P0[robotNumber]);
    qDebug() << epsilon;
    emit workFinished();
}

void SIVIA::runAll(vector<box>& T0,vector<Robot*> *rob, vector<iMatrix> &distance){
    int ny = distance.size();

    double err[] = {0.000, 0.001, 0.01, 0.01,0.00, 0.01,0.01, 0.001};
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
            Decremente(T0[i][2*j+1],T0[i][2*j+2], T0[i-1][2*j+1], T0[i-1][2*j+2], r->theta_v[i],r->speed_v[i]);
        }
    }
}


void SIVIA::fixPoint(box &X, iMatrix &distance){
    bool sortie=false;
    while (!sortie)
    {  box Xold(X);
        // Call the contracteur
        outerContractAll2(X,distance);
        if (X.IsEmpty())      sortie=true;
        if (decrease(Xold,X)<0.05) sortie=true;
    }
}

void SIVIA::doAllInOne(box X0, vector<vector<interval> >* dist){
    Init();
    // Clear Internal Data
    result.clear();
    P0.clear();
    contractor = &SIVIA::contract_and_draw;
    this->dist = *dist;
    SIVIA_f(X0);

}

void SIVIA::doContractState(box X0, vector<Robot*> *rob, vector<vector<vector<interval> > > *distance)
{
    Init();
    this->rob = rob;
    this->distance = distance;
    //contractor = &SIVIA::
    contractState(X0);
    //SIVIA_f(X0);
    emit workFinished();
}


void SIVIA::doStepR1(box X0, vector<Robot*> *rob, vector<vector<vector<interval> > > *distance){
    this->distance = distance;
    this->rob = rob;
    SIVIA_f(X0);
    emit workFinished();
}


box SIVIA::doContractOneByOne(box X, vector< vector< interval> > *dist){
    Init();
    result.clear();
    P0.clear();
    this->dist = *dist;
    // Copy initial position.
    for(uint i = 0; i < X.dim*0.5; i++){
        P0.push_back(box(X[2*i+1],X[2*i+2]));
    }
    currentRob = 1;
    contractor=&SIVIA::contract_and_drawOneRobot;
    //contractor=&SIVIA::wrapperContractOneRobot;
    //contractor=&SIVIA::contract_and_drawOneRobot;
    // DO sivia for each Robot.
    //for(uint j = 0; j < 10; j++){
    //currentRob = 1;
    for(uint i = 1; i < P0.size(); i++){
        result.clear();
        box X0(X[2*i+1],X[2*i+2]);
        emit drawBox(X0, YELLOWBOX);
        X0 = Inflate(X0, 0.2);
        SIVIA_f(X0);
        box Xr = getResult();
        if(Xr.IsEmpty()) emit drawBox(Xr, DARKBOX);
        P0[i] = Inter(P0[i],Xr);
        emit drawBox(Xr,DARKBLUEBOX );
        currentRob++;
    }
    //}

    box Xr(X.dim);
    for(uint i = 0; i < X.dim*0.5; i++){
        Xr[2*i+1] = P0[i][1];
        Xr[2*i+2] = P0[i][2];
    }
    return Xr;


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
        //qDebug()<<"k="<<k<< "box width "<< X.Width() << "stack size " << L.size() <<"Axe princ " << AxePrincipal(X) ;
        X=L.front();   L.pop_front();
        //emit drawBox(X,1);
        bool sortie=false;
        while (!sortie)
        {  box Xold(X);
            //emit drawBox(X,1);
            // Call the contracteur
            (this->*contractor)(X);
            if (X.IsEmpty())      sortie=true;
            if (decrease(Xold,X)<0.05) sortie=true;
        }

        if (!X.IsEmpty())
        {  if (X.Width()<epsilon)
            {
                emit drawBox(box(X[1],X[2]),YELLOWBOX);
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

    //qDebug() << "Number of boxes "<<k;
    //emit drawBox(X,7);
    //Rsivia->Save("paving");
    return 0;
}
//------------------------------------------------------------------------

void SIVIA::contractState(box& X){

    int ny = distance->size();
    int step = 2*rob->size();
    double err[] = {0.000, 0.01, 0.01, 0.01,0.00, 0.01,0.01, 0.01};
    // Forward propagation
    for(int i = 1; i < ny; i++){ // i => indice du temps
        // X(k) = f(X(k-1))
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Incremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2], X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2], r->theta_v[i],r->speed_v[i],err[j]);
        }
        // ******************************************************************************
        // HERE CALL THE CONTRACTOR TO FIND X_hat = G⁻1(Y) and make X = Inter(X, X_hat)
        // ******************************************************************************
        box Xtmp = X.extract(i*step, step);
        incr.push_back(Xtmp);
        box Xr = doContractOneByOne(Xtmp,&distance->at(i));
        if(Xr.IsEmpty()){
            qDebug() << "erreur empty" << Xr;
            //exit(-1);
            //break;
        } else {
            Xtmp = Inter(Xtmp, Xr);
            X.remplace(Xtmp,i*step );
        }
        cont.push_back(Xr);
        qDebug() << "position numero "<< i;
    }

    // Backward propagation
    for(int i = ny-1; i > 0; i--){ // i => indice du temps
        for(uint j =0; j < rob->size(); j++){ // j => indice Robot
            Robot* r = rob->at(j);
            Decremente(X[i*step + 2*j + 1],X[i*step + 2*j + 2], X[(i-1)*step + 2*j + 1],X[(i-1)*step + 2*j + 2], r->theta_v[i],r->speed_v[i]);
        }
    }




}


void SIVIA::contract_and_drawOneRobot(box &X){
    AContractorPtr inside = &SIVIA::innerContractOneRobot;
    AContractorPtr outside = &SIVIA::outerContractOneRobot;

    contract_and_draw(X,inside, false);
    if(X.IsEmpty())return ;
    contract_and_draw(X,outside, true);
    if (X.IsEmpty())  result.push_back(X);
}

void SIVIA::contract_and_draw(box &X, AContractorPtr contract, bool outside){
    vector<box>* dif;
    box Xold2(X);
    (this->*contract)(X);
    dif = diff(box(Xold2[1],Xold2[2]) , box(X[1],X[2]));
    for(int i = 0; i  < dif->size(); i++){
        box Xt = dif->at(i);
        if(outside == true)
            emit drawBox(Xt,OUTSIDEBOX);
        else
            emit drawBox(Xt,INSIDEBOX);
    }
    delete dif;
}

void SIVIA::contract_and_draw(box &X){
    vector<box>* dif;
    box Xold2(X);
    outerContractAll(X);
    for(uint r = 0; r < X.dim*0.5; r++){
        dif = diff(box(Xold2[2*r+1],Xold2[2*r+2]) , box(X[2*r+1],X[2*r+2]));
        for(int i = 0; i  < dif->size(); i++){
            box Xt = dif->at(i);
            emit drawBox(Xt,OUTSIDEBOX);
        }
        delete dif;
    }
    if(X.IsEmpty()){
        return ;
    }
    box Xold(X);
    innerContractAll(X);
    if (X.IsEmpty()){
        result.push_back(X);
    }
    for(uint r = 0; r < X.dim*0.5; r++){
        dif = diff(box(Xold[2*r+1],Xold[2*r+2]) , box(X[2*r+1],X[2*r+2]));
        for(uint i = 0; i  < dif->size(); i++){
            box Xt = dif->at(i);
            emit drawBox(Xt,INSIDEBOX);
        }
        delete dif;
    }
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
    for(uint r = 1; r < 0.5*X.dim; r++){
        for(uint i = 0; i < 0.5*X.dim; i++){
            box X1(X);
            if(i == r) continue;
            interval i1(dist[r][i]);
            CdiskExists(X1[2*r+1],X1[2*r+2],X1[2*i+1],X1[2*i+2],i1,false);
            if(X1.IsEmpty()) qDebug() << "empty result" << i << " " << r;
            L.push_back(X1);
        }
    }
    C_q_in(X, L.size()-N_outliers, L);
}

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

void SIVIA::innerContractAll(box& X){

    vector<box> L, L2;
    for(uint r = 1; r < 0.5*X.dim; r++){
        //box B0(X[2*r+1], X[2*r+2]);
        for(uint i = 0; i < 0.5*X.dim; i++){
            box X1(X);
            //box b0(X[2*r+1], X[2*r+2]);
            if(i == r) continue;
            interval i1(dist[r][i]);
            CdiskExists(X1[2*r+1], X1[2*r+2], X1[2*i+1], X1[2*i+2],i1,true);
            L.push_back(X1);
        }
//        box Xt((X[2*r+1], X[2*r+2]));
//        C_q_in(Xt, N_outliers +1, L);
//        L.clear();
//        L2.push_back(Xt);
//        if(L2[L2.size()-1] == B0) qDebug("same box => no contraction %d ",r);
//        if(Xt.IsEmpty()) qDebug("Empty result");
    }
//    for(int r = 1; r < 0.5*X.dim; r++){
//        X[2*r+1] = L2[r-1][1];
//        X[2*r+2] = L2[r-1][2];
//    }
    C_q_in(X, N_outliers +1, L);
}


//-----------------------------------------------------------------
//-------------------  CONTRACTOR ONE Robot    --------------------
//-----------------------------------------------------------------
void SIVIA::innerContractOneRobot(box& X){
    contractOneRobot(X,P0,&dist[currentRob][0],currentRob,true);
}


void SIVIA::outerContractOneRobot(box& X){
    contractOneRobot(X,P0,&dist[currentRob][0],currentRob,false);
}

void SIVIA::wrapperContractOneRobot(box& X){


    // Outer contraction
    contractOneRobot(X,P0,&dist[currentRob][0],currentRob,false);
    if(X.IsEmpty())return ; // X doesn't belong to the solution set

    // Inner contraction
    box Xold(X);
    contractOneRobot(X,P0,&dist[currentRob][0],currentRob,true);
    vector<box>* dif;
    dif = diff(Xold, X);
    for(int i = 0; i  < dif->size(); i++){
        result.push_back(dif->at(i));
    }
    delete dif;
}


/**
 * @brief Contract the position of one robot wrt the position of the others robot
 *
 * @param X             2-dimensional box which contains the position of the robot to contract
 * @param P             List of boxes which represent the centers of the circles
 * @param distances     Array of intervals containing the distances from the robotNumber and the others
 * @param robotNumber   Number of the robot to find its position
 * @param direction     If direction=false (resp. true) : inner (resp. outer) contractor
 */
void SIVIA::contractOneRobot(box& X, vector<box> &P,  interval * distances, int robotNumber, bool direction){

    vector<box> L;
    for(uint i = 0; i < P.size(); i++){
        if(i == robotNumber) continue;
        box X1(X);
        interval i1(distances[i]);
        CdiskExists(X1[1],X1[2],P[i][1],P[i][2],i1,direction);
        L.push_back(X1);
    }
    if(direction == false){ // See Morgan law
        C_q_in(X, L.size()-N_outliers, L);
    } else {
        C_q_in(X, N_outliers +1, L);
    }
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
    for(uint i= 0; i < result.size(); i++){
        box b = result.at(i);
        for(uint j = 1; j <= b.dim ; j++){
            if(b[j].inf == -999){
                qDebug() << "empty restult "<< i << " / " << result.size()-1 <<" " << result[i] << "----------------";
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

    if(X1.IsEmpty()) {
        res->push_back(X0);
        return res;
    }
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
