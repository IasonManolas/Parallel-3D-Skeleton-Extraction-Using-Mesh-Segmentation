#include "glwidget.h"

#include <GL/glx.h>
//#include <QOpenGLContext>

// parent needs to be there as an argument although it is not being used since
// in the .ui file there must exist a hierarchy between the objects
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
  delete defaultShader;
  delete axesShader;
  delete segmentShader;
  delete activeShader;
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

  // char *renderer = (char *)glGetString(GL_RENDERER);
  // char *version = (char *)glGetString(GL_VERSION);
  // char *vendor = (char *)glGetString(GL_VENDOR);

  // std::cout << "version:" << version << std::endl;
  // std::cout << "renderer:" << renderer << std::endl;
  // std::cout << "vendor:" << vendor << std::endl;

  initializeShaders();
  scene.initializeScene(); // has to be done here since glew needs to have been
                           // initialized

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
  glShadeModel(GL_SMOOTH);
  if (surfaceState == dontShowTriangles)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  else if (surfaceState == showTriangles)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  scene.Draw(activeShader, axesShader);
}
void GLWidget::mousePressEvent(QMouseEvent *event) {
  lastMousePos = QVector2D(event->localPos());
  if (mode == segmentsView) {
    if (event->button() == Qt::RightButton) {
      segmentSelection_signal(lastMousePos.x(), lastMousePos.y());
    }
  }
}

void GLWidget::showMeshSegments_signal() { scene.handle_showSegments(); }

void GLWidget::segmentSelection_signal(float mousePosX, float mousePosY) {
  scene.handle_segmentSelection(mousePosX, mousePosY, WIDTH, HEIGHT);
}

void GLWidget::segmentDeformation_signal(bool inflation) {
  if (inflation)
    scene.handle_meshInflation();
  else
    scene.handle_meshDeflation();
}

void GLWidget::cameraZoomChange_signal(float delta) {
  scene.handle_cameraZoomChange(delta);
}

void GLWidget::cameraReset_signal() { scene.handle_cameraReset(); }

void GLWidget::initializeShaders() {
  defaultShader =
      new Shader("../shaders/vertex.glsl", "../shaders/fragment.glsl");
  axesShader = new Shader("../shaders/axesvs.glsl", "../shaders/axesfs.glsl");
  segmentShader =
      new Shader("../shaders/vertex.glsl", "../shaders/segmfragment.glsl");
  activeShader = defaultShader;
}

void GLWidget::contraction_signal() {
  if (mode == defaultView)
    scene.handle_meshContraction();
  else if (mode == segmentsView)
    scene.handle_segmentContraction();
}
void GLWidget::connectivitySurgery_signal() {

  if (mode == defaultView)
    scene.handle_meshConnectivitySurgery();
  else if (mode == segmentsView)
    scene.handle_segmentConnectivitySurgery();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  QVector2D mouseOffset = QVector2D(event->localPos()) - lastMousePos;
  lastMousePos = QVector2D(event->localPos());

  if (event->buttons() == Qt::LeftButton) {
    if (mode == defaultView || mode == segmentsView) {
      // glm::vec2 mouseMoveOffset(mouseOffset.x(), mouseOffset.y());
      scene.handle_mouseMovement(mouseOffset); // TODO create a signal function
    }
  }
}

void GLWidget::wheelEvent(QWheelEvent *event) {

  if (!(event->modifiers() & Qt::ControlModifier))
    cameraZoomChange_signal(event->delta());
  else if ((event->modifiers() & Qt::ControlModifier) && mode == segmentsView) {
    segmentDeformation_signal(event->delta() > 0);
  }
}

void GLWidget::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {

  case Qt::Key_M:
    mode = defaultView;
    activeShader = defaultShader;
    break;

  case Qt::Key_S:
    mode = segmentsView;
    activeShader = segmentShader;
    showMeshSegments_signal();
    break;

  case Qt::Key_C:
    contraction_signal();
    break;
case Qt::Key_1:
    connectivitySurgery_signal();
    break;


    // case Qt::Key_A:
    //  if (mode == contractSegment) {
    //    if (selectedSegmentIndex == -1) {
    //      std::cout << "Please select a segment to contract." << std::endl;
    //    } else {
    //      // scene.contractSegment(selectedSegmentIndex);
    //      scene.P.updateMeshBuffers();
    //    }
    //    break;
    //  }
    // case Qt::Key_R:
    //  scene.contractMesh();
    //  scene.P.updateMeshBuffers();
    //  break;
  }
}
void GLWidget::keyReleaseEvent(QKeyEvent *event) {
  switch (event->key()) {}
}

void GLWidget::loadModel(std::__cxx11::string filename) {
  makeCurrent();
  activeShader = defaultShader;
  mode = defaultView;
  scene.loadMesh(filename);
}

void GLWidget::updateAxesState(bool state) {
  scene.handle_axesStateChange(state);
}

void GLWidget::updateMeshSurfaceState(bool state) {
  surfaceState = static_cast<meshSurfaceVizualization>(state);
}

void GLWidget::resetCamera() {
  std::cout << "camera reseted" << std::endl;

  cameraReset_signal();
}

void GLWidget::showVerticesStateChange(int state) {
  scene.handle_meshVerticesStateChange(state);
}

void GLWidget::saveModel(std::string destinationDirectory) {
  scene.handle_saveModel(destinationDirectory);
}
void GLWidget::saveSegment(std::string destinationDirectory) {
  scene.handle_saveSegment(destinationDirectory);
}
