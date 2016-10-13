#ifndef GLWIDGET_H
#define GLWIDGET_H


// Include GLM
#include <glm/glm.hpp>

//Include GLEW
#define GLEW_STATIC
#include <GL/glew.h>

//Qt includes
#include <QOpenGLWidget>
#include <QKeyEvent>
#include <QTimer>

//My headers
#include <shader.h>
#include <camera.h>

using namespace glm;

class GLWidget:public QOpenGLWidget
{

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:

void do_movement();
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
private:
    Shader *shaderObject;
    GLuint shaderProgram;
    GLuint VAO;
    QTimer timer;
    GLuint texture1;
    GLuint texture2;
    Camera *camObject;
};
#endif // GLWIDGET_H
