#ifndef GLWIDGET_H
#define GLWIDGET_H

// Include standard headers
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include <string>
// Include GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// Qt includes
//#include <QDebug>
#include <QKeyEvent>
#include <QOpenGLDebugLogger>
#include <QOpenGLWidget>
#include <QTime>
#include <QTimer>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// My headers
#include "camera.h"
#include "directionallight.h"
#include "material.h"
#include "mesh.h"
#include "scene.h"
#include "shader.h"

enum meshSurfaceVizualization { dontShowTriangles, showTriangles };
enum Mode { defaultView, segmentsView };

class GLWidget : public QOpenGLWidget {
  Q_OBJECT
public:
  explicit GLWidget(QWidget *parent);

private:
  ~GLWidget();
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void showMeshSegments_signal();
  void segmentSelection_signal(float, float);
  void contraction_signal();
void connectivitySurgery_signal();
  void showMeshVertices_signal();
  void segmentDeformation_signal(bool inflation);
  void cameraZoomChange_signal(float delta);
  void cameraReset_signal();
  void initializeShaders();

private:
  meshSurfaceVizualization surfaceState{dontShowTriangles};
  Scene scene{};
  Shader *defaultShader{nullptr};
  Shader *axesShader{nullptr};
  Shader *segmentShader{nullptr};
  Shader *activeShader{nullptr};
  QTimer timer{};
  QVector2D lastMousePos{0, 0};
  float fov{45};
  int WIDTH{800};
  int HEIGHT{600};
  Mode mode{defaultView};

public slots:
  void loadModel(std::string filename);
  void updateAxesState(bool state);
  void updateMeshSurfaceState(bool state);
  void resetCamera();
  void showVerticesStateChange(int state);
  void saveModel(std::string);
  void saveSegment(std::string);
};
#endif // GLWIDGET_H
