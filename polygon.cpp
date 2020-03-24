#include "polygon.h"

Polygon::Polygon(QList<Vertex> vertices)
{
    this->nb_vertex = vertices.size();
    GLfloat texCoords[] =
    {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
    };
    for (int i = 0; i < this->nb_vertex; ++i)
    {
        // coordonnées sommets
        //for (int j = 0; j < 3; j++)
            vertData.append(vertices[i].getX());
            vertData.append(vertices[i].getY());
            vertData.append(vertices[i].getZ());
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData.append(texCoords[j]);
    }
}
void Polygon::setColor(float r,float g, float b, float a)
{
    this->color = QVector4D(r,g,b,a);
}

void Polygon::display(QOpenGLShaderProgram *buffer)
{
    vbo.create();
    this->vbo.bind();
    this->vbo.allocate(vertData.constData(), vertData.count() * int(sizeof(GLfloat)));

    matrix.setToIdentity();
    matrix.translate(0.0,0.0,0.0);

    buffer->setUniformValue("modelMatrix", matrix);
    buffer->setUniformValue("particleSize", 1.0f);
    //QColor c(255,0,0,255);
    //GLfloat color[] = {255.0f, 0.0f, 0.0f, 200.0f};
    //QVector4D color(1.0f/this->position[1],1.0f/this->position[1],1.0f/this->position[1],1.0f);



    //glBindVertexArray(color);
    //buffer->setUniformValue("uv",color);

    buffer->setUniformValue("color",this->color);
    //buffer->setUniformValue("uv",color);
    buffer->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    buffer->setAttributeBuffer("in_uv", GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));
    buffer->enableAttributeArray("in_position");
    buffer->enableAttributeArray("in_uv");


    //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    //glEnable(GL_BLEND);
    //glDrawArrays(GL_LINE_LOOP, 0, this->nb_vertex);
    glDrawArrays(GL_POLYGON, 0, this->nb_vertex);
}
