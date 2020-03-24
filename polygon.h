#ifndef POLYGON_H
#define POLYGON_H

#include <QVector3D>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include "vertex.h"
class Polygon
{

public:
    Polygon(QList<Vertex> vertices);
    void display(QOpenGLShaderProgram * buffer);
    void setColor(float r,float g, float b, float a);
private:

    QOpenGLBuffer vbo;
    QVector4D color = QVector4D(1.0f,1.0f,1.0f,0.5f);

    QMatrix4x4 matrix;
    QVector<GLfloat> vertData;
    unsigned int nb_vertex;
};

#endif // POLYGON_H
