#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include <QVector3D>

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "directionallight.h"
#include "camera.h"
#include "axes.h"
#include "shader.h"
#include "mypolyhedron.h"

class Scene
{
public:
    float scaleFactor;
    Camera camera{glm::vec3(0.0f,0.0f,3.0f),glm::vec3(0.0f,0.0f,0.0f),glm::vec3(0.0f,1.0f,0.0f)};
    MyPolyhedron P;

    explicit Scene(){}

    void Draw(Shader* modelShader,Shader* axisShader)
    {
        setUniforms(modelShader,axisShader);
        modelShader->Use();
        P.Draw();
        if(showAxes)
        {
            axisShader->Use();
            sceneAxes.Draw();
        }
    }
    void updateProjectionMatrix(int w,int h)
    {
       projectionMatrix=glm::perspective(camera.fov,float(w)/float(h),nearPlane,farPlane);
    }

    void loadMesh(std::string filename)
    {
       P=MyPolyhedron(filename);
    }
    void setShowAxes(bool value)
    {
        showAxes=value;
    }

private:
    DirectionalLight light{glm::vec3(1.0f),glm::vec3(0.0f,0.0f,-1.0f)};
    Axes sceneAxes{};
    bool showAxes{false};
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
        P.setUniforms(modelShader);
        camera.setUniforms(modelShader);
        setProjectionMatrixUniform(modelShader);

        axisShader->Use();
        setProjectionMatrixUniform(axisShader);
        glUniformMatrix4fv(glGetUniformLocation(axisShader->programID,"model"),1,GL_FALSE,glm::value_ptr(glm::mat4(1.0f)));
        camera.setUniforms(axisShader);


    }
};

#endif // SCENE_H
