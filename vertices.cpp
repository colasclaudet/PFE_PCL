#include "vertices.h"

Vertices::Vertices(GLfloat size,QList<Vertex> vertices)
{
    this->listVertices = vertices;
    qDebug()<<"NB_VERTICES : "<<this->listVertices.size();
}

void Vertices::display(QOpenGLShaderProgram *buffer)
{
    QList<Vertex>::iterator i;
    i = listVertices.begin();
    while( i != listVertices.end() )
    {
            i->display(buffer);
            i++;
    }
}
