#ifndef DIRECTIONALLIGHT_H
#define DIRECTIONALLIGHT_H

#include <iostream>

#include <glm/vec3.hpp>
#include "shader.h"
#include "light.h"

class DirectionalLight:public Light
{
public:
    DirectionalLight(glm::vec3 lightColor,glm::vec3 pos,glm::vec3 dir):Light{lightColor,pos,dir}
    {
    }

    void changeLightDirection(const glm::vec3 newPosition)
    {
        position=newPosition;
        direction=glm::normalize(-position);
        std::cout<<"LIGHT:X="<<position.x<<" Y="<<position.y<<" Z="<<position.z<<std::endl;
    }
    void setUniforms(Shader* shader) const
    {
        glUniform3f(glGetUniformLocation(shader->programID, "light.ambient"),  ambient.x, ambient.y, ambient.z);
        glUniform3f(glGetUniformLocation(shader->programID, "light.diffuse"),  diffuse.x, diffuse.y, diffuse.z);
        glUniform3f(glGetUniformLocation(shader->programID, "light.specular"), specular.x,specular.y,specular.z);
        glUniform3f(glGetUniformLocation(shader->programID,"light.direction"),direction.x,direction.y,direction.z);
//        glUniform3f(glGetUniformLocation(shader->programID,"light.direction"),0.0f,0.0f,-1.0f);
//        glUniform3f(glGetUniformLocation(shader->programID,"light.position"),position.x,position.y,position.z);
    }

private:
    };

#endif // DIRECTIONALLIGHT_H
