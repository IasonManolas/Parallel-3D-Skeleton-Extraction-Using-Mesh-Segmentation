#ifndef GLWIDGET_H
#define GLWIDGET_H

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include <string>
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

enum meshSurfaceVizualization{dontShowTriangles,showTriangles};
class GLWidget:public QOpenGLWidget
{
Q_OBJECT
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
//    std::string filename{"../OpenGL_WithoutWrappers/Models/bunny.obj"};
    meshSurfaceVizualization surfaceState{dontShowTriangles};
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

public slots:
    void modelWasChosen(std::string filename);
    void updateAxesState(bool state);
    void updateMeshSurfaceState(bool state);
    void resetCamera();
};
#endif // GLWIDGET_H
