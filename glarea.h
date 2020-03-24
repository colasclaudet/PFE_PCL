// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019


#ifndef GLAREA_H
#define GLAREA_H

#include <QKeyEvent>
#include <QTimer>
#include <QElapsedTimer>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include "planes.h"
#include "vertices.h"
#include "polygons.h"

class GLArea : public QOpenGLWidget,
               protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLArea(QWidget *parent = nullptr);
    ~GLArea() override;

    void draw_plane(QVector3D P1,QVector3D P2,QVector3D P3,QVector3D P4);
    void draw_bounding_box(GLfloat xmax =0.0f, GLfloat ymax=0.0f, GLfloat zmax=0.0f, GLfloat xmin=0.0f, GLfloat ymin=0.0f, GLfloat zmin=0.0f);
    void addPlanes(QList<Plane> lplanes);
    void addVertex(QList<Vertex> lvertex);
    void addPolygon(QList<Polygon> lpoly);
    QList<Plane> list_plane;
    QList<Vertex> list_vertices;
    QList<Polygon> list_polygon;

protected slots:
    void onTimeout();

protected:
    void initializeGL() override;
    void doProjection();
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent *ev) override;
    void keyReleaseEvent(QKeyEvent *ev) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseReleaseEvent(QMouseEvent *ev) override;
    void mouseMoveEvent(QMouseEvent *ev) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    float xRot=20.0f, yRot=0.0f, zRot=0.0f;
    float xPos=0.0f,  yPos=0.0f, zPos=-100.0f;
    QTimer *timer = nullptr;
    QElapsedTimer elapsedTimer;
    float dt = 0;
    float windowRatio = 1.0f;
    QPoint lastPos;

    QOpenGLShaderProgram *program_boundingBox;
    QOpenGLShaderProgram *program_particule;
    QOpenGLShaderProgram *program_plane;
    //QOpenGLShaderProgram *program_sphere;
    QOpenGLBuffer vbo_box;
    QOpenGLBuffer vbo_particule;

    QOpenGLTexture *textures[2];

    void makeGLObjects();
    void tearGLObjects();

    //Qt3DRender::QMesh * mesh;

    float r_light = 0.02;
    float g_light = 0.02;
    float b_light = 0.05;

    float a_light = 1.0;

    Planes * planes;
    Vertices * vertices;
    Polygons * polygons;
};


#endif // GLAREA_H
