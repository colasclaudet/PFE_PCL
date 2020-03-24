#ifndef POLYGONS_H
#define POLYGONS_H
#include "polygon.h"

class Polygons
{
public:
    Polygons(QList<Polygon> _listPolygon);
    void display(QOpenGLShaderProgram * buffer);
private:
    QList<Polygon> listPolygon;
};

#endif // POLYGONS_H
