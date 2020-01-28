#include "plane.h"

Plane::Plane(QVector3D P1, QVector3D P2, QVector3D P3, QVector3D P4)
{
    GLfloat vertices[] =
    {
       P1[0],P1[1],P1[2],
       P2[0],P2[1],P2[2],
       P3[0],P3[1],P3[2],
       P3[0],P3[1],P3[2],
       P4[0],P4[1],P4[2],
       P1[0],P1[1],P1[2]
    };

    GLfloat texCoords[] =
    {
            0.0f, 0.0f,
            0.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 1.0f,
            1.0f, 0.0f,
            0.0f, 0.0f
    };

    for (int i = 0; i < 6; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData.append(vertices[i*3+j]);
        // coordonnées texture
        for (int j = 0; j < 2; j++)
            vertData.append(texCoords[i*2+j]);
    }
}

void Plane::display(QOpenGLShaderProgram *buffer)
{
    vbo.create();
    this->vbo.bind();
    this->vbo.allocate(vertData.constData(), vertData.count() * int(sizeof(GLfloat)));

    matrix.setToIdentity();
    //matrix.translate(this->position);

    buffer->setUniformValue("modelMatrix", matrix);
    buffer->setUniformValue("particleSize", 1);
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
    glDrawArrays(GL_TRIANGLES, 0, 6);
}
