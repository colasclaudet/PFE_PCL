#ifndef PLANE_H
#define PLANE_H
#include <QVector3D>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
class Plane
{
public:
    Plane(QVector3D P1, QVector3D P2, QVector3D P3, QVector3D P4);
    void display(QOpenGLShaderProgram * buffer);
private:

    QOpenGLBuffer vbo;
    QVector4D color = QVector4D(1.0f,1.0f,1.0f,0.5f);

    QMatrix4x4 matrix;
    QVector<GLfloat> vertData;
};

#endif // PLANE_H
