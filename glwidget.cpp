#include "glwidget.h"

#include <GL/glx.h>
#include <QOpenGLContext>
GLWidget::GLWidget(QWidget *parent) {

  QOpenGLWidget *widget = this;
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setVersion(4, 1);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setOption(QSurfaceFormat::DebugContext);

  widget->setFormat(format);

  setFocusPolicy(Qt::StrongFocus);
}

GLWidget::~GLWidget() {
  makeCurrent();
  delete modelShader;
  delete axesShader;
  disconnect(&timer, SIGNAL(timeout()), this, SLOT(update()));
}
void GLWidget::initializeGL() {

  // Set this to true so GLEW knows to use a modern approach to retrieving
  // function pointers and extensions
  glewExperimental = GL_TRUE;
  // Initialize GLEW to setup the OpenGL Function pointers
  GLenum err = glewInit();
  if (GLEW_OK != err) {
    qDebug() << "[Error] GLEW failed to initialize. "
             << (const char *)glewGetErrorString(err);
  }

  glViewport(0, 0, WIDTH, HEIGHT);

  glEnable(GL_DEPTH_TEST);

  char *renderer = (char *)glGetString(GL_RENDERER);
  char *version = (char *)glGetString(GL_VERSION);
  char *vendor = (char *)glGetString(GL_VENDOR);

  std::cout << "version:" << version << std::endl;
  std::cout << "renderer:" << renderer << std::endl;
  std::cout << "vendor:" << vendor << std::endl;
  // I have to constructors for class Shader in order
  // to use the "real" one after the opengl context is active
  // IS THIS SYNTAX OS dependant? build dir must be in the same folder(aka
  // Projects) as the sources(aka OpenGL_WithoutWrappers)
  modelShader = new Shader("../Thesis/shaders/vertex.glsl",
                           "../Thesis/shaders/fragment.glsl");
  axesShader = new Shader("../Thesis/shaders/axesvs.glsl",
                          "../Thesis/shaders/axesfs.glsl");

  scene.loadMesh("../Thesis/Models/bunny.obj");
  scene.intersectionSphere.load("../Thesis/Models/icosahedron.obj",
                                scene.P.averageEdgeLength / 5);

  connect(&timer, SIGNAL(timeout()), this, SLOT(update()));
  timer.start(30);
}
void GLWidget::resizeGL(int w, int h) {
  WIDTH = w;
  HEIGHT = h;
  scene.updateProjectionMatrix(w, h);
}

void GLWidget::paintGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (surfaceState == dontShowTriangles)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  else if (surfaceState == showTriangles)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  scene.Draw(modelShader, axesShader);
}
void GLWidget::mousePressEvent(QMouseEvent *event) {
  // initialize mouse position
  lastMousePos = QVector2D(event->localPos());
  if (mode == pickVertex &&
      scene.rayIntersectsPolyhedron(lastMousePos.x(), lastMousePos.y(), WIDTH,
                                    HEIGHT)) {
    scene.showIntersection = true;
  } else if (mode == pickSegment &&
             scene.rayIntersectsPolyhedron(lastMousePos.x(), lastMousePos.y(),
                                           WIDTH, HEIGHT)) {
    scene.showIntersection = false;
  }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  QVector2D mouseOffset = QVector2D(event->localPos()) - lastMousePos;
  lastMousePos = QVector2D(event->localPos());

  if (event->buttons() == Qt::LeftButton) {
    if (mode == view) {
      glm::vec2 mouseMoveOffset(mouseOffset.x(), mouseOffset.y());
      scene.processMouseMovement(mouseOffset);
    }
    //    else if (mode == pick) {
    //      scene.rayIntersectsPolyhedron(lastMousePos.x(), lastMousePos.y(),
    //      WIDTH,
    //                                    HEIGHT);
    //    }
  }
}

void GLWidget::wheelEvent(QWheelEvent *event) {
  scene.camera.processWheelMovement(event->delta());
}

#include <thread>
void GLWidget::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {

  case Qt::Key_M: {
    mode = static_cast<Mode>(mode ^ 1);

    break;
  }
  case Qt::Key_S: {
    std::thread t(&MyPolyhedron::segmentMesh, scene.P);
    t.join();
    break;
  }
  }
}
// void GLWidget::keyReleaseEvent(QKeyEvent *event)
//{
//    switch(event->key()){

//        case Qt::Key_M:
//            break;

//    }

//}

void GLWidget::modelWasChosen(std::__cxx11::string filename) {
  makeCurrent();
  scene.loadMesh(filename);
}

void GLWidget::updateAxesState(bool state) { scene.setShowAxes(state); }

void GLWidget::updateMeshSurfaceState(bool state) {
  surfaceState = static_cast<meshSurfaceVizualization>(state);
}

void GLWidget::resetCamera() {
  std::cout << "camera reseted" << std::endl;
  scene.camera.resetCamera();
  scene.light.changeLightDirection(scene.camera.getPosition());
}

void GLWidget::handleLoggedMessage(QOpenGLDebugMessage message) {
  qDebug() << message;
}
