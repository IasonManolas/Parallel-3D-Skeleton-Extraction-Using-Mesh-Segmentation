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
#include <QTime>
#include <QVector3D>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//My headers
#include "shader.h"
#include "camera.h"
#include "model.h"
#include "directionallight.h"
#include "material.h"
#include "polyhedronbuilder.h"
#include "polyhedronprocessor.h"
#include "scene.h"

class GLWidget:public QOpenGLWidget
{

public:
    explicit GLWidget(QWidget *parent);
    ~GLWidget();
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent *event);
private:
    Shader* modelShader{nullptr};
    Shader* axesShader{nullptr};
    QTimer timer{};
    QVector2D lastMousePos{0,0};
    Scene scene{};
    Polyhedron P{};
    PolyhedronProcessor PP{};
    Kernel::Iso_cuboid_3 bbox{};
    QVector3D bboxCenter{};
    float scaleFactor{1};
    float fov{45};
    int WIDTH{800};
    int HEIGHT{600};


};
#endif // GLWIDGET_H
