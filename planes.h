#ifndef PLANES_H
#define PLANES_H

#include <QVector>
#include "plane.h"
class Planes
{
public:
    Planes(QList<Plane> _listPlane);
    void display(QOpenGLShaderProgram * buffer);
private:
    QList<Plane> listPlane;
};

#endif // PLANES_H
