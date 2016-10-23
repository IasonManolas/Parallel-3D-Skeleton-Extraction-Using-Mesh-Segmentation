#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent):shaderObject(),camObject(),fov(45.0f),WIDTH(800),HEIGHT(600) //I have to constructors for class Shader in order
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
    delete axesShader;
    delete ourModel;
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

    //I have to constructors for class Shader in order
    //to use the "real" one after the opengl context is active
    //IS THIS SYNTAX OS dependant? build dir must be in the same folder(aka Projects) as the sources(aka OpenGL_WithoutWrappers)
    shaderObject=new Shader("../OpenGL_WithoutWrappers/shaders/vertex.glsl","../OpenGL_WithoutWrappers/shaders/fragment.glsl");
    axesShader=new Shader("../OpenGL_WithoutWrappers/shaders/simplevs.glsl","../OpenGL_WithoutWrappers/shaders/simplefs.glsl");
    light=DirectionalLight(glm::vec3(1.0f),glm::vec3(0.0f,0.0f,-1.0f));
    material=Material(glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.5f,0.5f,0.0f),glm::vec3(0.6f,0.6f,0.5f),128*0.25);

    ourModel=new Model("../OpenGL_WithoutWrappers/Models/bunny.obj");
    PolyhedronBuilder<HalfedgeDS> builder(ourModel->meshes[0]);
    P.delegate(builder);
    PP=PolyhedronProcessor(P);

   bbox=PP.getBbox();
   bboxCenter={(bbox.xmax()+bbox.xmin())/2.0,(bbox.ymax()+bbox.ymin())/2.0,(bbox.zmax()+bbox.zmin())/2.0};
   float maxDim=std::max({bbox.xmax()-bbox.xmin(),bbox.ymax()-bbox.ymin(),bbox.zmax()-bbox.zmin()});
   float camZ=0.5/tan(fov/2.0);
   scaleFactor=1.0/maxDim;
   camObject=Camera(glm::vec3(0.0f,0.0f,3.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f));
    connect(&timer,SIGNAL(timeout()),this,SLOT(update()));
            timer.start(30);
}
void GLWidget::resizeGL(int w, int h)
{
    WIDTH=w;
    HEIGHT=h;
}

void GLWidget::paintGL()
{
    glClearColor(0.0f,0.0f,0.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    shaderObject->Use();

    light.setUniforms(shaderObject);
    material.setUniforms(shaderObject);
    glUniform3f(glGetUniformLocation(shaderObject->programID, "viewPos"),camObject.getPosition().x,camObject.getPosition().y, camObject.getPosition().z);

   glm::mat4 projection = glm::perspective(fov, (float)WIDTH/(float)HEIGHT, 0.1f, 1000.0f);
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
   glm::mat4 view = camObject.getViewMat();
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "view"), 1, GL_FALSE, glm::value_ptr(view));

   glm::mat4 model;
   model = glm::scale(model, glm::vec3(scaleFactor, scaleFactor, scaleFactor));
   model = glm::translate(model,glm::vec3(-bboxCenter.x(),-bboxCenter.y(),-bboxCenter.z()) );
//   model=glm::mat4(1.0f);
   glUniformMatrix4fv(glGetUniformLocation(shaderObject->programID, "model"), 1, GL_FALSE, glm::value_ptr(model));

   ourModel->Draw(shaderObject);

  axesShader->Use();
   glUniformMatrix4fv(glGetUniformLocation(axesShader->programID, "view"), 1, GL_FALSE, glm::value_ptr(view));
   glUniformMatrix4fv(glGetUniformLocation(axesShader->programID, "projection"), 1, GL_FALSE, glm::value_ptr(projection));


  axes.Draw(axesShader);
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
        camObject.processMouseMovement(mouseOffset);
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
   camObject.processWheelMovement(event->delta());

}

