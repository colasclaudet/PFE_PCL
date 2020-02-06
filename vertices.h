#ifndef VERTICES_H
#define VERTICES_H

#include "vertex.h"
class Vertices
{
public:
    Vertices(GLfloat size,QList<Vertex> vertices);
    void display(QOpenGLShaderProgram * buffer);
private:
    QList<Vertex> listVertices;
};

#endif // VERTICES_H
