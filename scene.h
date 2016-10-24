#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include <QVector3D>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "directionallight.h"
#include "model.h"
#include "camera.h"
#include "axes.h"
#include "shader.h"

class Scene
{
public:
    Scene(){}
    //fix Scene(int)
    Scene(int a):axes(),camera(glm::vec3(0.0f,0.0f,3.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f)),
        light(glm::vec3(1.0f),glm::vec3(0.0f,0.0f,-1.0f)),nearPlane(0.1f),farPlane(1000.0f),
        projectionMatrix(glm::mat4(1.0f))
    {
        model=Model("../OpenGL_WithoutWrappers/Models/bunny.obj");
    }

    void Draw(Shader* modelShader,Shader* axisShader)
    {
        setUniforms(modelShader,axisShader);
        modelShader->Use();
        model.Draw();
        axisShader->Use();
        axes.Draw();
    }
    void updateProjectionMatrix(int w,int h)
    {
       projectionMatrix=glm::perspective(camera.fov,float(w)/float(h),nearPlane,farPlane);
    }

    float scaleFactor;
    QVector3D bboxCenter;

    Model model;
    Camera camera;

  private:

    DirectionalLight light;
    Axes axes;

    void setProjectionMatrixUniform(Shader* shader)
    {
        glUniformMatrix4fv(glGetUniformLocation(shader->programID,"projection"),1,GL_FALSE,glm::value_ptr(projectionMatrix));
    }

    void setUniforms(Shader* modelShader,Shader* axisShader)
    {
        modelShader->Use();
        light.setUniforms(modelShader);
        model.setUniforms(modelShader);
        camera.setUniforms(modelShader);
        setProjectionMatrixUniform(modelShader);
        glm::mat4 model;
        model = glm::scale(model, glm::vec3(scaleFactor, scaleFactor, scaleFactor));
        model = glm::translate(model,glm::vec3(-bboxCenter.x(),-bboxCenter.y(),-bboxCenter.z()) );
        glUniformMatrix4fv(glGetUniformLocation(modelShader->programID, "model"), 1, GL_FALSE, glm::value_ptr(model));

        axisShader->Use();
        setProjectionMatrixUniform(axisShader);
        glUniformMatrix4fv(glGetUniformLocation(axisShader->programID,"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
        camera.setUniforms(axisShader);


    }

    glm::mat4 projectionMatrix;

    float nearPlane;
    float farPlane;
};

#endif // SCENE_H
