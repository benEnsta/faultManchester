#include "repere.h"
#include <QDebug>
#include <QtSvg/QSvgGenerator>
//#include "map.h"

repere::repere(QObject *parent) :
    QObject(parent)
{
}
//--------------------------------------------------------------------------------------------------
repere::repere(QObject *parent,QGraphicsView* G,double xmin1, double xmax1,double ymin1, double ymax1) :
    QObject(parent)
{    Scene=new QGraphicsScene(this);
     this->G = G;
     G->setScene(Scene);
     xmin=xmin1; xmax=xmax1; ymin=ymin1; ymax=ymax1;
     Scene->setSceneRect(0,0,G->geometry().width()-3,G->geometry().height()-3);
     DrawBox(xmin,xmax,ymin,ymax,QPen(Qt::red),QBrush(Qt::NoBrush));
}
//--------------------------------------------------------------------------------------------------
double repere::xToPix(double x)
{   double echx = Scene->width()/(xmax-xmin);
    return (x-xmin)*echx;
}
//--------------------------------------------------------------------------------------------------
double repere::yToPix(double y)
{
    double echy = Scene->height()/(ymax-ymin);
    return Scene->height()-(y-ymin)*echy;
}
//--------------------------------------------------------------------------------------------------
void repere::Clean()
{
    Scene->clear();
}
//--------------------------------------------------------------------------------------------------
void repere::DrawBox(double xmin,double xmax,double ymin,double ymax, QPen pen1, QBrush brush1)
{       QPolygonF box1;
        box1 << QPointF(xToPix(xmin),yToPix(ymin))<< QPointF(xToPix(xmax),yToPix(ymin))
             << QPointF(xToPix(xmax),yToPix(ymax))<< QPointF(xToPix(xmin),yToPix(ymax));
        //QGraphicsPolygonItem *P;
        Scene->addPolygon(box1,pen1,brush1);
 }
//---------------------------------------------------------------------------------------------------
void repere::DrawHok(float *tab,float* angles, double x, double y, double theta){
       QVector<QPointF> point;
       point.push_back(QPointF(xToPix(x),yToPix(y)));
       for(int i = 0; i < 726; i++){
           double xp = xToPix(x + tab[i]*1*cos(angles[i] + theta));
           double yp = yToPix(y + tab[i]*1*sin(angles[i] + theta));
           point.push_back(QPointF(xp,yp));
       }
       point.push_back(QPointF(xToPix(x),yToPix(y)));
       Scene->addPolygon(point,QPen(Qt::red), QBrush(Qt::gray));
}

void repere::centerOn(double x, double y, double width, double height)
{
    Scene->setSceneRect(x,y,G->geometry().width()-3,G->geometry().height()-3);
}
//---------------------------------------------------------------------------------------------------
void repere::DrawHok_contract(double xv, double yv, double thetav,
                              vector<float> &dist, vector<float> &alpha, vector<float> &gamma, vector<bool> &contract){
    for(int i = 0; i < alpha.size(); i++ ){

        float xtmp  = xv+dist[i]*cos(thetav+alpha[i]);
        float ytmp  = yv+dist[i]*sin(thetav+alpha[i]);
        if( contract[i] == true){
            DrawLine(xv,yv,xtmp,ytmp,QPen(Qt::green));    // faisceau
        }else{
            DrawLine(xv,yv,xtmp,ytmp,QPen(Qt::red));    // faisceau
        }
        //Rworld->DrawLine(xtmp,y[i-step/2],x[i],y[i],QPen(Qt::red));
        //Rworld->DrawArrow(xtmp,ytmp,cos(thetav+t2),sin(thetav+t2),0.1,QPen(Qt::gray)); //angle d'impact
        //DrawArrow(xtmp,ytmp,cos(thetav+valpha[i-step/2]+vgamma[i-step/2]),sin(thetav+valpha[i-step/2]+vgamma[i-step/2]),0.1,QPen(Qt::black)); //angle d'impact

    }
}

//--------------------------------------------------------------------------------------------------
void repere::DrawArrow(double x1,double y1,double dx,double dy, double r, QPen pen1)
{       QPolygonF Arrow1;
        double x2=x1+dx;
        double y2=y1+dy;
        double a=3.14-1.0;
        double px=x2+r*(cos(a)*dx-sin(a)*dy);    // cot de pointe
        double py=y2+r*(sin(a)*dx+cos(a)*dy);
        double qx=x2+r*(cos(-a)*dx-sin(-a)*dy);  // autre cot de pointe
        double qy=y2+r*(sin(-a)*dx+cos(-a)*dy);
        Scene->addLine(xToPix(x1),yToPix(y1),xToPix(x2),yToPix(y2),pen1);
        Scene->addLine(xToPix(x2),yToPix(y2),xToPix(px),yToPix(py),pen1);
        Scene->addLine(xToPix(x2),yToPix(y2),xToPix(qx),yToPix(qy),pen1);

 }
//--------------------------------------------------------------------------------------------------
void repere::DrawEllipse(double cx, double cy, double r, QPen pen1, QBrush brush1)
{
     Scene->addEllipse(xToPix(cx-r),yToPix(cy+r),xToPix(cx+r)-xToPix(cx-r),yToPix(cy-r)-yToPix(cy+r),pen1,brush1);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawLine(double x1,double y1,double x2,double y2, QPen pen1)
{       Scene->addLine(xToPix(x1),yToPix(y1),xToPix(x2),yToPix(y2),pen1);
}
//--------------------------------------------------------------------------------------------------
/*void repere::DrawMap(Map map)
{
    for(int i = 0; i < map.walls.size()-1; i++){
        QPoint p1 = map.walls[i];
        QPoint p2 = map.walls[i+1];
        DrawLine(p1.x(),p1.y(),p2.x(),p2.y(),QPen(Qt::blue,3));
    }
}*/
//--------------------------------------------------------------------------------------------------
void repere::DrawPolygone(double x,double y,double theta,vector<double> X, vector<double> Y, QPen pen1, QBrush brush1)
{       QPolygonF poly1;
        for (int k=0;k<(int)X.size();k++)
        {
            double x1=x+cos(theta)*X[k]-sin(theta)*Y[k];
            double y1=y+sin(theta)*X[k]+cos(theta)*Y[k];
            poly1 << QPointF(xToPix(x1),yToPix(y1));
        }
        //QGraphicsPolygonItem *P;
        Scene->addPolygon(poly1,pen1,brush1);
 }
//--------------------------------------------------------------------------------------------------
void repere::DrawRobot(double x,double y,double theta, double s, QPen pen, QBrush brush)
{   vector<double> X,Y;
    X.push_back(0*s); Y.push_back(-1*s);
    X.push_back(3*s); Y.push_back(0*s);
    X.push_back(0*s); Y.push_back(1*s);
    DrawPolygone(x,y,theta,X,Y,pen, brush);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawText(QString s, int n)
{       for (int k=1;k<=n;k++)
        s="\r\n"+s;
        //Pour la syntaxe:  int a; double p; s=QString("a=%1,\r\n p=%2").arg(k).arg(p);
        Scene->addText(s);
}
//--------------------------------------------------------------------------------------------------
void repere::Save(QString nom)
{
    QImage *image = new QImage(QSize(1000,1000),QImage::Format_ARGB32);
    image->fill(QColor(Qt::white).rgb());
    QPainter *pngPainter = new QPainter(image);
    pngPainter->setRenderHint(QPainter::Antialiasing);
    Scene->render(pngPainter);
    pngPainter->end();
    image->save(nom+".png","PNG",100);
}
//--------------------------------------------------------------------------------------------------
void repere::SaveSVG(QString nom)
{
    QSvgGenerator generator;
    generator.setFileName(nom);
    generator.setSize(QSize(1000, 1000));
    generator.setViewBox(QRect(0, 0, 1000, 1000));
    generator.setTitle(tr("SVG Generator Example Drawing"));
    generator.setDescription(tr("An SVG drawing created by the SVG Generator "
                                "Example provided with Qt."));
    QPainter painter;
    painter.begin(&generator);
    Scene->render(&painter);
    painter.end();
}
//--------------------------------------------------------------------------------------------------

