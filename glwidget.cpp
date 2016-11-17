#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
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
    delete modelShader;
    delete axesShader;
    disconnect(&timer,SIGNAL(timeout()),this,SLOT(update()));
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

    char *renderer=(char*)glGetString(GL_RENDERER);
    char *version = (char*)glGetString(GL_VERSION);
    char *vendor= (char*)glGetString(GL_VENDOR);

    std::cout <<"version:"<<version<<std::endl ;
    std::cout <<"renderer:"<<renderer<<std::endl ;
    std::cout <<"vendor:"<<vendor<<std::endl ;
    //I have to constructors for class Shader in order
    //to use the "real" one after the opengl context is active
    //IS THIS SYNTAX OS dependant? build dir must be in the same folder(aka Projects) as the sources(aka OpenGL_WithoutWrappers)
    modelShader=new Shader("../OpenGL_WithoutWrappers/shaders/vertex.glsl","../OpenGL_WithoutWrappers/shaders/fragment.glsl");
    axesShader=new Shader("../OpenGL_WithoutWrappers/shaders/simplevs.glsl","../OpenGL_WithoutWrappers/shaders/simplefs.glsl");

//    scene.loadMesh("../OpenGL_WithoutWrappers/Models/Alien Bust.obj");
//    scene.loadMesh("../OpenGL_WithoutWrappers/Models/bunny.obj");
//    scene.loadMesh("../OpenGL_WithoutWrappers/Models/bunny_low.obj");

    connect(&timer,SIGNAL(timeout()),this,SLOT(update()));
            timer.start(30);
}
void GLWidget::resizeGL(int w, int h)
{
    WIDTH=w;
    HEIGHT=h;
    scene.updateProjectionMatrix(w,h);
}

void GLWidget::paintGL()
{


       glClearColor(0.0f,0.0f,0.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(surfaceState==dontShowTriangles)
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    else if(surfaceState==showTriangles)
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

    scene.Draw(modelShader,axesShader);
}
void GLWidget::mousePressEvent(QMouseEvent *event)
{
    //initialize mouse position
    lastMousePos=QVector2D(event->localPos());
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    QVector2D mouseOffset=QVector2D(event->localPos())-lastMousePos;
    lastMousePos=QVector2D(event->localPos());

    if(event->buttons()== Qt::LeftButton)
    {
        glm::vec2 mouseMoveOffset(mouseOffset.x(),mouseOffset.y());
        scene.camera.processMouseMovement(mouseOffset);
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
  scene.camera.processWheelMovement(event->delta());
}

void GLWidget::modelWasChosen(std::__cxx11::string filename)
{
    scene.loadMesh(filename);
}

void GLWidget::updateAxesState(bool state)
{
    scene.setShowAxes(state);
}

void GLWidget::updateMeshSurfaceState(bool state)
{
    surfaceState=static_cast<meshSurfaceVizualization>(state);
}

void GLWidget::resetCamera()
{
    scene.camera.resetCamera();
}

