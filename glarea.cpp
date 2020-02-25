// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019
//from TP3 colas claudet animation et rendu
#include "glarea.h"
#include <QDebug>
#include <QSurfaceFormat>
#include <QMatrix4x4>

GLArea::GLArea(QWidget *parent) :
    QOpenGLWidget(parent)
{
    QSurfaceFormat sf;
    sf.setDepthBufferSize(24);
    sf.setSamples(16);
    setFormat(sf);

    setEnabled(true);                   // événements clavier et souris
    setFocusPolicy(Qt::StrongFocus);    // accepte focus
    setFocus();                         // donne le focus

    timer = new QTimer(this);
    timer->setInterval(50);           // msec
    connect (timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start();
    elapsedTimer.start();
}


GLArea::~GLArea()
{
    delete timer;

    // Contrairement aux méthodes virtuelles initializeGL, resizeGL et repaintGL,
    // dans le destructeur le contexte GL n'est pas automatiquement rendu courant.
    makeCurrent();
    tearGLObjects();
    doneCurrent();
}

void GLArea::draw_bounding_box(GLfloat xmax, GLfloat ymax, GLfloat zmax, GLfloat xmin, GLfloat ymin, GLfloat zmin)
{
    //float tailleaq = 15.0f;
    //from TP2 colas claudet animation et rendu

    GLfloat vertices_aq[] = {
           xmin,ymin,zmin,
           xmax,ymin,zmin,
           xmin,ymin,zmax,
           xmax,ymin,zmax,
           xmin,ymax,zmin,
           xmax,ymax,zmin,
           xmin,ymax,zmax,
           xmax,ymax,zmax,

           xmin,ymin,zmin,
           xmin,ymin,zmax,
           xmin,ymax,zmin,
           xmin,ymax,zmax,
           xmax,ymin,zmin,
           xmax,ymin,zmax,
           xmax,ymax,zmin,
           xmax,ymax,zmax,

           xmin,ymin,zmin,
           xmin,ymax,zmin,
           xmax,ymin,zmin,
           xmax,ymax,zmin,
           xmin,ymin,zmax,
           xmin,ymax,zmax,
           xmax,ymin,zmax,
           xmax,ymax,zmax
        };
    GLfloat texCoords_aq[] = {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
        };

    QVector<GLfloat> vertData_aq;
    for (int i = 0; i < 24; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_aq.append(vertices_aq[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData_aq.append(texCoords_aq[i*2+j]);
    }
    vbo_sol.create();
    vbo_sol.bind();
    vbo_sol.allocate(vertData_aq.constData(), vertData_aq.count() * int(sizeof(GLfloat)));

}

void GLArea::addPlanes(QList<Plane> lplanes)
{
    this->list_plane = lplanes;
    makeGLObjects();
}

void GLArea::addVertex(QList<Vertex> lvertex)
{
    this->list_vertices = lvertex;
    makeGLObjects();
}


void GLArea::initializeGL()
{

    initializeOpenGLFunctions();
    glClearColor(r_light,g_light,b_light,a_light);
    glEnable(GL_DEPTH_TEST);

    makeGLObjects();
    //load3DMod("name");

    // shader du sol
    program_sol = new QOpenGLShaderProgram(this);
    //program_sol->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/simple.vsh");
    //program_sol->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/simple.fsh");

    program_sol->addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/simple.vsh");
    program_sol->addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/simple.fsh");
    if (! program_sol->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_sol->log();
    }
    program_sol->setUniformValue("texture", 0);

    // shader de billboard
    program_particule = new QOpenGLShaderProgram(this);
    //program_particule->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/billboard.vsh");
    //program_particule->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/billboard.fsh");

    program_particule->addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/billboard_m.vsh");
    program_particule->addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/billboard_m.fsh");
    if (! program_particule->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_particule->log();
    }
    program_particule->setUniformValue("texture", 0);

    program_box = new QOpenGLShaderProgram(this);
    program_box->addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/billboard.vsh");
    program_box->addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/billboard.fsh");
    if (! program_box->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_box->log();
    }
    program_box->setUniformValue("texture", 0);


}


void GLArea::makeGLObjects()
{
    // Création du sol
    draw_bounding_box();
    if(list_plane.size()>0)
    {
        this->planes = new Planes(this->list_plane);
    }

    if(list_vertices.size()>0)
    {
        this->vertices = new Vertices(0.5,this->list_vertices);
    }

    /*float tailleSol = 20.0f;

    GLfloat vertices_sol[] = {
       -tailleSol, 0.0f,-tailleSol,
       -tailleSol, 0.0f, tailleSol,
        tailleSol, 0.0f, tailleSol,
        tailleSol, 0.0f, tailleSol,
        tailleSol, 0.0f,-tailleSol,
       -tailleSol, 0.0f,-tailleSol
    };

    GLfloat texCoords_sol[] = {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
        };

    QVector<GLfloat> vertData_sol;
    for (int i = 0; i < 6; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_sol.append(vertices_sol[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData_sol.append(texCoords_sol[i*2+j]);
    }

    vbo_sol.create();
    vbo_sol.bind();
    vbo_sol.allocate(vertData_sol.constData(), vertData_sol.count() * int(sizeof(GLfloat)));
    */

    // Création d'une particule de fumée


    // Création de textures
    //QImage image_sol(":/textures/solplanche.jpg");
    //QImage image_sol(":/textures/ground.jpg");
    QImage image_sol("../textures/ground.jpg");
    if (image_sol.isNull())
        qDebug() << "load image ground.jpg failed";
    textures[0] = new QOpenGLTexture(image_sol);
    //QImage image_sol(":/textures/puff.png");
    QImage image_particule("../textures/puff.png");
    if (image_particule.isNull())
        qDebug() << "load image puff.png failed";
    textures[1] = new QOpenGLTexture(image_particule);
}


void GLArea::tearGLObjects()
{
    vbo_sol.destroy();
    vbo_particule.destroy();
    for (int i = 0; i < 2; i++)
        delete textures[i];
}




void GLArea::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    windowRatio = float(w) / h;
}



void GLArea::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Matrice de projection
    QMatrix4x4 projectionMatrix;
    projectionMatrix.perspective(45.0f, windowRatio, 1.0f, 1000.0f);

    // Matrice de vue (caméra)
    QMatrix4x4 viewMatrix;
    viewMatrix.translate(xPos, yPos, zPos);
    viewMatrix.rotate(xRot, 1, 0, 0);
    viewMatrix.rotate(yRot, 0, 1, 0);
    viewMatrix.rotate(zRot, 0, 0, 1);

    // Affichage du sol
    vbo_sol.bind();
    program_sol->bind(); // active le shader program du sol

    QMatrix4x4 modelMatrixSol;
    modelMatrixSol.translate(0.0f, 0.0f, 0.0f);
    program_sol->setUniformValue("projectionMatrix", projectionMatrix);
    program_sol->setUniformValue("viewMatrix", viewMatrix);
    program_sol->setUniformValue("modelMatrix", modelMatrixSol);
    program_sol->setUniformValue("color",QVector4D(1.0,1.0,1.0,1.0));

    program_sol->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    program_sol->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    program_sol->enableAttributeArray("in_position");
    program_sol->enableAttributeArray("in_uv");

    //textures[0]->bind();
    glDrawArrays(GL_LINES, 0, 24);
    //textures[0]->release();

    program_sol->disableAttributeArray("in_position");
    program_sol->disableAttributeArray("in_uv");
    program_sol->release();


    // Affichage d'une particule
    //vbo_particule.bind();
    program_particule->bind(); // active le shader program des particules

    QMatrix4x4 modelMatrixParticule;

    //glDepthMask(GL_FALSE);//à décocher plus tard

    program_particule->setUniformValue("projectionMatrix", projectionMatrix);
    program_particule->setUniformValue("viewMatrix", viewMatrix);
    //program_particule->setUniformValue("modelMatrix", modelMatrixParticule);
    //program_particule->setUniformValue("particleSize", 1.0f);

    //program_particule->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    //program_particule->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    //program_particule->enableAttributeArray("in_position");
    //program_particule->enableAttributeArray("in_uv");

    //textures[1]->bind();
    //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    //glEnable(GL_BLEND);
    //glDrawArrays(GL_TRIANGLES, 0, 6);
    //glDisable(GL_BLEND);
    //textures[1]->release();
    if(list_vertices.size()>0)
    {
        vertices->display(program_particule);
    }
    program_particule->disableAttributeArray("in_position");
    program_particule->disableAttributeArray("in_uv");
    program_particule->release();
    //glDepthMask(GL_TRUE);

    program_box->bind();
    program_box->setUniformValue("projectionMatrix", projectionMatrix);
    program_box->setUniformValue("viewMatrix", viewMatrix);
    //textures[2]->bind();

    if(list_plane.size()>0)
    {
        planes->display(program_box);
    }

    //textures[2]->release();
    program_box->disableAttributeArray("in_position");
    program_box->disableAttributeArray("in_uv");
    program_box->release();

    //glDepthMask(GL_TRUE);//à décocher plus tard

}


void GLArea::keyPressEvent(QKeyEvent *ev)
{
    float da = 1.0f;

    switch(ev->key()) {
        case Qt::Key_A :
            xRot -= da;
            update();
            break;

        case Qt::Key_Q :
            xRot += da;
            update();
            break;

        case Qt::Key_Z :
            yRot -= da;
            update();
            break;

        case Qt::Key_S :
            yRot += da;
            update();
            break;

        case Qt::Key_E :
            zRot -= da;
            update();
            break;

        case Qt::Key_D :
            zRot += da;
            update();
            break;
        case Qt::Key_T :
            initializeGL();
            update();
            break;
        case Qt::Key_Up:
            zPos +=0.5;
            update();
            break;
        case Qt::Key_Down:
            zPos -=0.5;
            update();
            break;
        case Qt::Key_B :
        this->r_light = 0.0;
        this->g_light = 0.0;
        this->b_light = 0.0;
        this->a_light = 0.0;
        initializeGL();

        update();
        break;
    }
}


void GLArea::keyReleaseEvent(QKeyEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->text();
}


void GLArea::mousePressEvent(QMouseEvent *ev)
{
    lastPos = ev->pos();
}


void GLArea::mouseReleaseEvent(QMouseEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->x() << ev->y() << ev->button();
}


void GLArea::mouseMoveEvent(QMouseEvent *ev)
{
    int dx = ev->x() - lastPos.x();
    int dy = ev->y() - lastPos.y();

    if (ev->buttons() & Qt::LeftButton) {
        xRot += dy;
        yRot += dx;
        update();
    } else if (ev->buttons() & Qt::RightButton) {
        xPos += dx/10.0f;
        yPos -= dy/10.0f;
        update();
    } else if (ev->buttons() & Qt::MidButton) {
        xPos += dx/10.0f;
        zPos += dy;
        update();
    }

    lastPos = ev->pos();
}

void GLArea::wheelEvent(QWheelEvent *event)
{
    if(event->delta()>0)
    {
        zPos+=1.5;
    }
    else
    {
        zPos-=1.5;
    }


    event->accept();
}

void GLArea::onTimeout()
{
    static qint64 old_chrono = elapsedTimer.elapsed(); // static : initialisation la première fois et conserve la dernière valeur
    qint64 chrono = elapsedTimer.elapsed();
    dt = (chrono - old_chrono) / 1000.0f;
    old_chrono = chrono;
    update();
}
