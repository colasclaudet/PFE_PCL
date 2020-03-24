
//basé sur TP3 colas claudet animation et rendu
#include "glarea.h"
#include <QDebug>
#include <QSurfaceFormat>
#include <QMatrix4x4>
/**
 * @brief GLArea::GLArea - constructeur du openGL widget
 * @param parent - classe parent
 */
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

/**
 * @brief GLArea::~GLArea - destructeur du openGL widget
 */
GLArea::~GLArea()
{
    delete timer;
    // Contrairement aux méthodes virtuelles initializeGL, resizeGL et repaintGL,
    // dans le destructeur le contexte GL n'est pas automatiquement rendu courant.
    makeCurrent();
    tearGLObjects();
    doneCurrent();
}

/**
 * @brief GLArea::draw_bounding_box - initialisation de la boite englobante du nuage de point
 * @param xmax
 * @param ymax
 * @param zmax
 * @param xmin
 * @param ymin
 * @param zmin
 */
void GLArea::draw_bounding_box(GLfloat xmax, GLfloat ymax, GLfloat zmax, GLfloat xmin, GLfloat ymin, GLfloat zmin)
{
    GLfloat vertices_box[] = {
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
    GLfloat texCoords_box[] = {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
        };

    QVector<GLfloat> vertData_box;
    for (int i = 0; i < 24; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_box.append(vertices_box[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData_box.append(texCoords_box[i*2+j]);
    }
    vbo_box.create();
    vbo_box.bind();
    vbo_box.allocate(vertData_box.constData(), vertData_box.count() * int(sizeof(GLfloat)));

}
/**
 * @brief GLArea::addPlanes - Ajout de la liste de plan à la classe pour initialisation
 * @param lplane - liste de plan
 */
void GLArea::addPlanes(QList<Plane> lplanes)
{
    this->list_plane = lplanes;
    makeGLObjects();
}
/**
 * @brief GLArea::addVertex - Ajout de la liste de vertex à la classe pour initialisation
 * @param lvertex - liste de vertex
 */
void GLArea::addVertex(QList<Vertex> lvertex)
{
    //à changer => plutot faire des push backs
    this->list_vertices = lvertex;
    makeGLObjects();
}
/**
 * @brief GLArea::addPolygon - Ajout de la liste de polygone à la classe pour initialisation
 * @param lvertex - liste de polygone
 */
void GLArea::addPolygon(QList<Polygon> lpoly)
{
    this->list_polygon = lpoly;
    makeGLObjects();
}

/**
 * @brief GLArea::initializeGL - initialisation de l'environnement 3D
 */
void GLArea::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(r_light,g_light,b_light,a_light);
    glEnable(GL_DEPTH_TEST);

    makeGLObjects();

    // shader de la bounding box
    program_boundingBox = new QOpenGLShaderProgram(this);

    program_boundingBox->addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/simple.vsh");
    program_boundingBox->addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/simple.fsh");
    if (! program_boundingBox->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_boundingBox->log();
    }
    program_boundingBox->setUniformValue("texture", 0);

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

    program_plane = new QOpenGLShaderProgram(this);
    program_plane->addShaderFromSourceFile(QOpenGLShader::Vertex, "../shaders/billboard.vsh");
    program_plane->addShaderFromSourceFile(QOpenGLShader::Fragment, "../shaders/billboard.fsh");
    if (! program_plane->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_plane->log();
    }
    program_plane->setUniformValue("texture", 0);
}

/**
 * @brief GLArea::makeGLObjects - initialisation des objets
 */
void GLArea::makeGLObjects()
{
    // Création de la boite englobante
    draw_bounding_box();
    if(list_plane.size()>0)
    {
        this->planes = new Planes(this->list_plane);
    }

    if(list_vertices.size()>0)
    {
        this->vertices = new Vertices(0.5,this->list_vertices);
    }
    if(list_polygon.size()>0)
    {
        this->polygons = new Polygons(this->list_polygon);
    }

    // Création d'une particule de fumée

    // Création de textures
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

/**
 * @brief GLArea::tearGLObjects - destruction des objets
 */
void GLArea::tearGLObjects()
{
    vbo_box.destroy();
    vbo_particule.destroy();
    for (int i = 0; i < 2; i++)
        delete textures[i];
}

/**
 * @brief GLArea::resizeGL - modification de la taille de la fenêtre
 * @param w - width
 * @param h - height
 */
void GLArea::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    windowRatio = float(w) / h;
}

/**
 * @brief GLArea::paintGL - affichage des objets 3D
 */
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

    // Affichage de la bounding box
    vbo_box.bind();
    program_boundingBox->bind(); // active le shader program de la bounding box

    QMatrix4x4 modelMatrixSol;
    modelMatrixSol.translate(0.0f, 0.0f, 0.0f);
    program_boundingBox->setUniformValue("projectionMatrix", projectionMatrix);
    program_boundingBox->setUniformValue("viewMatrix", viewMatrix);
    program_boundingBox->setUniformValue("modelMatrix", modelMatrixSol);
    program_boundingBox->setUniformValue("color",QVector4D(1.0,1.0,1.0,1.0));

    program_boundingBox->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    program_boundingBox->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    program_boundingBox->enableAttributeArray("in_position");
    program_boundingBox->enableAttributeArray("in_uv");

    //textures[0]->bind();
    glDrawArrays(GL_LINES, 0, 24);
    //textures[0]->release();

    program_boundingBox->disableAttributeArray("in_position");
    program_boundingBox->disableAttributeArray("in_uv");
    program_boundingBox->release();

    program_particule->bind(); // active le shader program des particules

    QMatrix4x4 modelMatrixParticule;

    //glDepthMask(GL_FALSE);//à décocher plus tard

    program_particule->setUniformValue("projectionMatrix", projectionMatrix);
    program_particule->setUniformValue("viewMatrix", viewMatrix);
    if(list_vertices.size()>0)
    {
        vertices->display(program_particule);
    }
    program_particule->disableAttributeArray("in_position");
    program_particule->disableAttributeArray("in_uv");
    program_particule->release();
    //glDepthMask(GL_TRUE);

    program_plane->bind();
    program_plane->setUniformValue("projectionMatrix", projectionMatrix);
    program_plane->setUniformValue("viewMatrix", viewMatrix);
    //textures[2]->bind();

    if(list_plane.size()>0)
    {
        planes->display(program_plane);
    }
    if(list_polygon.size()>0)
    {
        polygons->display(program_plane);
    }
    //textures[2]->release();
    program_plane->disableAttributeArray("in_position");
    program_plane->disableAttributeArray("in_uv");
    program_plane->release();
    //glDepthMask(GL_TRUE);//à décocher plus tard
}

/**
 * @brief GLArea::keyPressEvent - gestion des événements clavier
 * @param ev - événement clavier
 */
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

/**
 * @brief GLArea::keyReleaseEvent - gestion des touches clavier relachées
 * @param ev - événement clavier
 */
void GLArea::keyReleaseEvent(QKeyEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->text();
}

/**
 * @brief GLArea::mousePressEvent - gestion des clics souris
 * @param ev - événement souris
 */
void GLArea::mousePressEvent(QMouseEvent *ev)
{
    lastPos = ev->pos();
}

/**
 * @brief GLArea::mouseReleaseEvent - gestion des clics souris relachés
 * @param ev - événement souris
 */
void GLArea::mouseReleaseEvent(QMouseEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->x() << ev->y() << ev->button();
}

/**
 * @brief GLArea::mouseMoveEvent - gestion des mouvements souris
 * @param ev - événement souris
 */
void GLArea::mouseMoveEvent(QMouseEvent *ev)
{
    int dx = ev->x() - lastPos.x();
    int dy = ev->y() - lastPos.y();
    if (ev->buttons() & Qt::LeftButton)
    {
        xRot += dy;
        yRot += dx;
        update();
    }
    else if (ev->buttons() & Qt::RightButton)
    {
        xPos += dx/10.0f;
        yPos -= dy/10.0f;
        update();
    }
    else if (ev->buttons() & Qt::MidButton)
    {
        xPos += dx/10.0f;
        zPos += dy;
        update();
    }
    lastPos = ev->pos();
}
/**
 * @brief GLArea::wheelEvent - Gestion de la molette souris
 * @param ev - événement souris
 */
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
/**
 * @brief GLArea::onTimeout - gestion de la position des objets en fonction du temps par raffraichissement
 */
void GLArea::onTimeout()
{
    static qint64 old_chrono = elapsedTimer.elapsed(); // static : initialisation la première fois et conserve la dernière valeur
    qint64 chrono = elapsedTimer.elapsed();
    dt = (chrono - old_chrono) / 1000.0f;
    old_chrono = chrono;
    update();
}
