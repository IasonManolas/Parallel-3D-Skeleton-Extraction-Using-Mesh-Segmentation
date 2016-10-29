#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include <QVector3D>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "directionallight.h"
#include "mesh.h"
#include "camera.h"
#include "axes.h"
#include "shader.h"
#include "mypolyhedron.h"

class Scene
{
public:
    float scaleFactor;
    QVector3D bboxCenter;
    Mesh mesh{};
    Camera camera{glm::vec3(0.0f,0.0f,3.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f)};
    MyPolyhedron P;

    Scene(){}
    explicit Scene(char* path):mesh(path),P(mesh){}

    void Draw(Shader* modelShader,Shader* axisShader)
    {
        setUniforms(modelShader,axisShader);
        modelShader->Use();
        mesh.Draw();
//        P.Draw();
        axisShader->Use();
        sceneAxes.Draw();
    }
    void updateProjectionMatrix(int w,int h)
    {
       projectionMatrix=glm::perspective(camera.fov,float(w)/float(h),nearPlane,farPlane);
    }

  private:

    DirectionalLight light{glm::vec3(1.0f),glm::vec3(0.0f,0.0f,-1.0f)};
    Axes sceneAxes{};
    glm::mat4 projectionMatrix{glm::mat4(1.0f)};
    float nearPlane{0.1};
    float farPlane{100};

    void setProjectionMatrixUniform(Shader* shader)
    {
        glUniformMatrix4fv(glGetUniformLocation(shader->programID,"projection"),1,GL_FALSE,glm::value_ptr(projectionMatrix));
    }
    void setUniforms(Shader* modelShader,Shader* axisShader)
    {
        modelShader->Use();
        light.setUniforms(modelShader);
        mesh.setUniforms(modelShader);
        camera.setUniforms(modelShader);
        setProjectionMatrixUniform(modelShader);

        axisShader->Use();
        setProjectionMatrixUniform(axisShader);
        glUniformMatrix4fv(glGetUniformLocation(axisShader->programID,"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
        camera.setUniforms(axisShader);


    }
};

#endif // SCENE_H
