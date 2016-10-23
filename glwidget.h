#ifndef GLWIDGET_H
#define GLWIDGET_H

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//Include GLEW
#define GLEW_STATIC
#include <GL/glew.h>

//Qt includes
#include <QOpenGLWidget>
#include <QKeyEvent>
#include <QTimer>
#include <QVector2D>
#include <QDebug>
#include <QOpenGLContext>
#include <QCoreApplication>
#include <QTime>
#include <QResizeEvent>
#include <QVector3D>
#include <QMatrix4x4>

#include <CGAL/Point_3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <SOIL.h>

//My headers
#include "shader.h"
#include "camera.h"
#include "model.h"
#include "directionallight.h"
#include "material.h"
#include "polyhedronbuilder.h"
#include "polyhedronprocessor.h"

class GLWidget:public QOpenGLWidget
{

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent *event);
private:
    Shader* shaderObject;
    QTimer timer;
    Camera camObject;
    QVector2D lastMousePos;
    Model* ourModel;
    DirectionalLight light;
    Material material;
    Polyhedron P;
    PolyhedronProcessor PP;
    Kernel::Iso_cuboid_3 bbox;
    QVector3D bboxCenter;
   float scaleFactor;
   float fov;
// Window dimensions
  int WIDTH;
  int HEIGHT;


};
#endif // GLWIDGET_H
