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
	delete edgeShader;
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

	char *renderer = (char *)glGetString(GL_RENDERER);
	char *version = (char *)glGetString(GL_VERSION);
	char *vendor = (char *)glGetString(GL_VENDOR);

	std::cout << "version:" << version << std::endl;
	std::cout << "renderer:" << renderer << std::endl;
	std::cout << "vendor:" << vendor << std::endl;

	initializeShaders();
	scene.initializeScene();  // has to be done here since glew needs to
				  // have been
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
	glClearColor(0, 0, 0, 1);
	// glClearColor(0.43f, 0.72f, 1.0f, 1.0f);
	// Enable blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);
	// glLineWidth(10);
	if (surfaceState == fillSurface)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	else if (surfaceState == fillSurface_and_showWireframe) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		scene.Draw(activeShader, axesShader, edgeShader, defaultShader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else if (surfaceState == showWireframe) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	scene.Draw(activeShader, axesShader, edgeShader, defaultShader);
}
void GLWidget::mousePressEvent(QMouseEvent *event) {
	lastMousePos = QVector2D(event->localPos());
	if (mode == segmentsView) {
		if (event->button() == Qt::RightButton) {
			segmentSelection_signal(lastMousePos.x(),
						lastMousePos.y());
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
	axesShader =
	    new Shader("../shaders/axesvs.glsl", "../shaders/axesfs.glsl");
	segmentShader = new Shader("../shaders/vertex.glsl",
				   "../shaders/segmfragment.glsl");
	edgeShader =
	    new Shader("../shaders/edgevs.glsl", "../shaders/edgefs.glsl");

	activeShader = defaultShader;
}

void GLWidget::cgalSkeletonization_signal() {
	if (mode == defaultView) {
		scene.handle_meshContraction(true, true);
	} else if (mode == segmentsView) {
		scene.handle_segmentContraction(true, true);
	}
}

void GLWidget::contraction_signal() {
	if (mode == defaultView) {
		if (contractionMode == automatic) {
			scene.handle_meshContraction(true, false);
		} else {
			scene.handle_meshContraction(false, false);
		}
	} else if (mode == segmentsView) {
		if (contractionMode == automatic) {
			scene.handle_segmentContraction(true, false);
		} else {
			scene.handle_segmentContraction(false, false);
		}
	}
}
void GLWidget::reverseContraction_signal() {
	if (mode == defaultView) scene.handle_meshContractionReversing();
}
void GLWidget::connectivitySurgery_signal() {
	if (mode == defaultView)
		scene.handle_meshConnectivitySurgery();
	else if (mode == segmentsView)
		scene.handle_segmentConnectivitySurgery();
}

void GLWidget::refinementEmbedding_signal() {
	if (mode == defaultView)
		scene.handle_meshRefinementEmbedding();
	else if (mode == segmentsView)
		scene.handle_segmentRefinementEmbedding();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event) {
	QVector2D mouseOffset = QVector2D(event->localPos()) - lastMousePos;
	lastMousePos = QVector2D(event->localPos());

	if (event->buttons() == Qt::LeftButton) {
		if (mode == defaultView || mode == segmentsView) {
			// glm::vec2
			// mouseMoveOffset(mouseOffset.x(),
			// mouseOffset.y());
			scene.handle_mouseMovement(
			    mouseOffset);  // TODO create a
					   // signal function
		}
	}
}

void GLWidget::wheelEvent(QWheelEvent *event) {
	if (!(event->modifiers() & Qt::ControlModifier))
		cameraZoomChange_signal(event->delta());
	else if ((event->modifiers() & Qt::ControlModifier) &&
		 mode == segmentsView) {
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
		case Qt::Key_R:
			reverseContraction_signal();
			break;
		case Qt::Key_1:
			contraction_signal();
			break;
		case Qt::Key_2:
			makeCurrent();
			connectivitySurgery_signal();
			doneCurrent();
			break;
		// case Qt::Key_3:
		//	refinementEmbedding_signal();
		//	break;
		case Qt::Key_3:
			scene.M.runSkeletonizationMethods();
			break;
		case Qt::Key_4:
			cgalSkeletonization_signal();
			break;
		case Qt::Key_P:
			scene.M.parallelSkeletonization();
			break;
	}
}
void GLWidget::keyReleaseEvent(QKeyEvent *event) {
	switch (event->key()) {}
}

void GLWidget::loadModel(std::__cxx11::string filename) {
	activeShader = defaultShader;
	mode = defaultView;
	makeCurrent();
	scene.loadMesh(filename);
	doneCurrent();
}

void GLWidget::updateAxesState(bool state) {
	scene.handle_axesStateChange(state);
}

void GLWidget::updateMeshSurfaceState(int state) {
	surfaceState = static_cast<meshSurfaceVizualization>(state);
}

void GLWidget::resetCamera() { cameraReset_signal(); }

void GLWidget::showVerticesStateChange(int state) {
	scene.handle_meshVerticesStateChange(state);
}

void GLWidget::saveModel(std::string destinationDirectory) {
	scene.handle_saveModel(destinationDirectory);
}
void GLWidget::saveSegment(std::string destinationDirectory) {
	scene.handle_saveSegment(destinationDirectory);
}

void GLWidget::updateContractionVolumeThreshold(int newThreshold) {
	scene.M.MC.setVolumeThreshold(std::pow(10.0, newThreshold));
}

void GLWidget::updateContractionMode(bool newState) {
	if (newState)
		contractionMode = automatic;
	else
		contractionMode = manual;
}

void GLWidget::clearSkeleton() { scene.handle_clearSkeleton(); }
