#ifndef DIRECTIONALLIGHT_H
#define DIRECTIONALLIGHT_H

#include <iostream>

#include <glm/vec3.hpp>
#include "shader.h"
#include "light.h"

class DirectionalLight:public Light
{
public:
    DirectionalLight(glm::vec3 lightColor,glm::vec3 dir):Light(lightColor),direction(dir)
    {
    }

    void setUniforms(Shader* shader) const
    {
        glUniform3f(glGetUniformLocation(shader->programID, "light.ambient"),  ambient.x, ambient.y, ambient.z);
        glUniform3f(glGetUniformLocation(shader->programID, "light.diffuse"),  diffuse.x, diffuse.y, diffuse.z);
        glUniform3f(glGetUniformLocation(shader->programID, "light.specular"), specular.x,specular.y,specular.z);
        glUniform3f(glGetUniformLocation(shader->programID,"light.direction"),direction.x,direction.y,direction.z);
    }

private:
    glm::vec3 direction{0,0,-1};
    };

#endif // DIRECTIONALLIGHT_H
