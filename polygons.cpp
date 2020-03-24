#include "polygons.h"

Polygons::Polygons(QList<Polygon> _listPolygon)
{
    this->listPolygon = _listPolygon;
    qDebug()<<"NB_POLYGONS : "<<this->listPolygon.size();
}

void Polygons::display(QOpenGLShaderProgram *buffer)
{
    QList<Polygon>::iterator i;
    i = listPolygon.begin();
    while( i != listPolygon.end() )
    {
        i->display(buffer);
        i++;
    }
    //qDebug()<<"display";
}
