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
    Scene(char* path)
    {
        model=Model(path);
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

    Model model{};
    Camera camera{glm::vec3(0.0f,0.0f,3.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f)};

  private:

    DirectionalLight light{glm::vec3(1.0f),glm::vec3(0.0f,0.0f,-1.0f)};
    Axes axes{};

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

    glm::mat4 projectionMatrix{glm::mat4(1.0f)};

    float nearPlane{0.1};
    float farPlane{100};
};

#endif // SCENE_H
