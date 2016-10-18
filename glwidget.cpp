#include "glwidget.h"
// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <QDebug>
#include <QOpenGLContext>
#include <QCoreApplication>
#include <QTime>

// GLM Mathematics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
//SOIL Image Loader
#include <SOIL.h>

// Window dimensions
#define  WIDTH GLuint(800)
#define  HEIGHT GLuint(600)

GLWidget::GLWidget(QWidget *parent):shaderObject(),camObject() //I have to constructors for class Shader in order
  //to use the "real" one after the opengl context is active
{

    QOpenGLWidget *widget=this;
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 1);
    format.setProfile(QSurfaceFormat::CoreProfile);
    widget->setFormat(format);

    setFocusPolicy(Qt::StrongFocus);
}

GLWidget::~GLWidget()
{
    makeCurrent();
    delete shaderObject;
    //delete lampShaderObject;
    delete camObject;
    delete ourModel;
}
void GLWidget::initializeGL()
{

    // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
    glewExperimental = GL_TRUE;
    // Initialize GLEW to setup the OpenGL Function pointers
    GLenum err = glewInit();
    if( GLEW_OK != err ){
    qDebug() << "[Error] GLEW failed to initialize. " << (const char*)glewGetErrorString(err);
    }
    glViewport(0,0,WIDTH,HEIGHT);

    glEnable(GL_DEPTH_TEST);

    //I have to constructors for class Shader in order
    //to use the "real" one after the opengl context is active
    //IS THIS SYNTAX OS dependant? build dir must be in the same folder(aka Projects) as the sources(aka OpenGL_WithoutWrappers)
    shaderObject=new Shader("../OpenGL_WithoutWrappers/shaders/vertex.glsl","../OpenGL_WithoutWrappers/shaders/fragment.glsl");
    //lampShaderObject=new Shader("../OpenGL_WithoutWrappers/shaders/vertex.glsl","../OpenGL_WithoutWrappers/shaders/lightfragment.glsl");
     //Create camera
    camObject=new Camera(glm::vec3(0.0f,0.0f,5.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f));

    ourModel=new Model("../OpenGL_WithoutWrappers/nanosuit/nanosuit.obj");
       connect(&timer,SIGNAL(timeout()),this,SLOT(update()));
            timer.start(30);



}
void GLWidget::resizeGL(int w, int h)
{

}

void GLWidget::paintGL()
{
    //Clear the colorbuffer
    glClearColor(0.1f,0.1f,0.1f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shaderObject->Use();
    // Transformation matrices
   glm::mat4 projection = glm::perspective(45.0f, (float)WIDTH/(float)HEIGHT, 0.1f, 100.0f);
   glm::mat4 view = camObject->getViewMat();
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "view"), 1, GL_FALSE, glm::value_ptr(view));

   // Draw the loaded model
   glm::mat4 model;
   model = glm::translate(model, glm::vec3(0.0f, -1.75f, 0.0f)); // Translate it down a bit so it's at the center of the scene
   model = glm::scale(model, glm::vec3(0.2f, 0.2f, 0.2f));	// It's a bit too big for our scene, so scale it down
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "model"), 1, GL_FALSE, glm::value_ptr(model));
   ourModel->Draw(shaderObject);
}

void GLWidget::keyPressEvent(QKeyEvent *event)//what does this mean?: If you reimplement this handler, it is very important that you call the base class implementation if you do not act upon the key.
{

}

void GLWidget::keyReleaseEvent(QKeyEvent *event)
{
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    //initialize mouse position
    lastMousePos=QVector2D(event->localPos());
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{

}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    QVector2D mouseOffset=QVector2D(event->localPos())-lastMousePos;
    lastMousePos=QVector2D(event->localPos());

    if(event->buttons()== Qt::LeftButton)
    {
        glm::vec2 mouseMoveOffset(mouseOffset.x(),mouseOffset.y());
        camObject->processMouseMovement(mouseOffset);
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
   camObject->processWheelMovement(event->delta());

}

