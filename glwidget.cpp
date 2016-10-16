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
    delete lampShaderObject;
    delete camObject;
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
    lampShaderObject=new Shader("../OpenGL_WithoutWrappers/shaders/vertex.glsl","../OpenGL_WithoutWrappers/shaders/lightfragment.glsl");
     //Create camera
    camObject=new Camera(glm::vec3(0.0f,0.0f,5.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f));

    GLfloat vertices[] = {
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
    };

    GLuint VBO,EBO;

    glGenVertexArrays(1,&VAO);
    glGenBuffers(1,&VBO);
    glGenBuffers(1,&EBO);

    //Bind the Vertex Array Object first,then bind and set vertex buffers and attribute pointers (WHY?)
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(vertices),vertices,GL_STATIC_DRAW);

    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(GLfloat),(GLvoid*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(GLfloat),(GLvoid*)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0); // Unbind VAO

   glGenVertexArrays(1,&lightVAO);
   glBindVertexArray(lightVAO);

   glBindBuffer(GL_ARRAY_BUFFER,VBO);

   glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(GLfloat),(GLvoid*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER,0);


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

    GLint viewPosLoc=glGetAttribLocation(shaderObject->programID,"viewPos");
    glm::vec3 camPos=camObject->getPosition();
    glUniform3f(viewPosLoc,camPos.x,camPos.y,camPos.z);

    GLint matAmbientLoc  = glGetUniformLocation(shaderObject->programID, "material.ambient");
    GLint matDiffuseLoc  = glGetUniformLocation(shaderObject->programID, "material.diffuse");
    GLint matSpecularLoc = glGetUniformLocation(shaderObject->programID, "material.specular");
    GLint matShineLoc    = glGetUniformLocation(shaderObject->programID, "material.shininess");
    glUniform3f(matAmbientLoc,  1.0f, 0.5f, 0.31f);
    glUniform3f(matDiffuseLoc,  1.0f, 0.5f, 0.31f);
    glUniform3f(matSpecularLoc, 0.5f, 0.5f, 0.5f);
    glUniform1f(matShineLoc,    32.0f);

    glm::vec3 lightDirection(-0.2f,-0.2f,-1.0f);
    GLint lightAmbientLoc  = glGetUniformLocation(shaderObject->programID, "light.ambient");
    GLint lightDiffuseLoc  = glGetUniformLocation(shaderObject->programID, "light.diffuse");
    GLint lightSpecularLoc = glGetUniformLocation(shaderObject->programID, "light.specular");
    GLint lightDirectionLoc = glGetUniformLocation(shaderObject->programID,"light.direction");
    glUniform3f(lightAmbientLoc,  0.2f, 0.2f, 0.2f);
    glUniform3f(lightDiffuseLoc,  0.5f, 0.5f, 0.5f); // Let's darken the light a bit to fit the scene
    glUniform3f(lightSpecularLoc, 1.0f, 1.0f, 1.0f);
    glUniform3f(lightDirectionLoc,lightDirection.x,lightDirection.y,lightDirection.z);

//    std::cout<<"camPos.x="<<camPos.x<<" camPos.y="<<camPos.y<<" camPos.z="<<camPos.z<<std::endl;

    // Camera/View transformation
    glm::mat4 view;
    view = camObject->getViewMat();

    // Projection
    glm::mat4 projection;
    projection = glm::perspective(45.0f, (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f);

    // Get the uniform locations
    GLint modelLoc = glGetUniformLocation(shaderObject->programID, "model");
    GLint viewLoc = glGetUniformLocation(shaderObject->programID, "view");
    GLint projLoc = glGetUniformLocation(shaderObject->programID, "projection");

    // Pass the matrices to the shader
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));


    glBindVertexArray(VAO);
    // Calculate the model matrix for each object and pass it to shader before drawing
    glm::mat4 model;
    model = glm::translate(model,glm::vec3(0.0f,0.0f,0.0f));
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glDrawArrays(GL_TRIANGLES, 0,36);
    glBindVertexArray(0);

//    //Light Cube drawing
//    lampShaderObject->Use();

//    // Get the uniform locations
//    modelLoc = glGetUniformLocation(lampShaderObject->programID, "model");
//    viewLoc = glGetUniformLocation(lampShaderObject->programID, "view");
//    projLoc = glGetUniformLocation(lampShaderObject->programID, "projection");

//    // Pass the matrices to the shader
//    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
//    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

//    glBindVertexArray(lightVAO);
//    model=glm::mat4();
//    model=glm::translate(model,lightDirection);
//    model=glm::scale(model,glm::vec3(0.2f));
//    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
//    glDrawArrays(GL_TRIANGLES, 0,36);
//    glBindVertexArray(0);
    //remember to create a destroy widget function for delete Shader, DeleteVertexArrays and DeleteBuffers

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

