#include "planes.h"

Planes::Planes(QList<Plane> _listPlane)
{
    this->listPlane = _listPlane;
}

void Planes::display(QOpenGLShaderProgram *buffer)
{
    QList<Plane>::iterator i;
    i = listPlane.begin();
    while( i != listPlane.end() )
    {
            i->display(buffer);
            i++;
    }
}
