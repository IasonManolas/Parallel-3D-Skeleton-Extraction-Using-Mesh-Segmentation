#ifndef AXES_H
#define AXES_H

#define GLEW_STATIC
#include <GL/glew.h>

#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>

#include "shader.h"
class Axes
{
public:
    Axes(){
        modelMat=glm::mat4(1.0f);
    }

    void Draw()
    {
        drawPreparation();

        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES,0,6);
        glBindVertexArray(0);
    }

    void setPosition(const float x,const float y,const float z)
    {
        modelMat=glm::translate(modelMat,glm::vec3(-x,-y,-z));
    }

    glm::mat4 getModelMat() const;

private:
    void drawPreparation()
    {
        glGenVertexArrays(1,&VAO);
        glGenBuffers(1,&VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER,VBO);
        glBufferData(GL_ARRAY_BUFFER,sizeof(vertices),vertices,GL_STATIC_DRAW);

        //set position attribute
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(GLfloat),(GLvoid*)0);
        glEnableVertexAttribArray(0);
        //set color attribute
        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(GLfloat),(GLvoid*)(3*sizeof(GLfloat)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);

    }



    GLfloat vertices[36]={
      0.0f,0.0f,0.0f,	1.0f,0.0f,0.0f,
      1.0f,0.0f,0.0f,	1.0f,0.0f,0.0f,

      0.0f,0.0f,0.0f,	0.0f,1.0f,0.0f,
      0.0f,1.0f,0.0f,	0.0f,1.0f,0.0f,

      0.0f,0.0f,0.0f,	0.0f,0.0f,1.0f,
      0.0f,0.0f,1.0f,	0.0f,0.0f,1.0f
    };

    GLuint VBO,VAO;
    glm::mat4 modelMat;
};

#endif // AXES_H

inline glm::mat4 Axes::getModelMat() const
{
return modelMat;
}
