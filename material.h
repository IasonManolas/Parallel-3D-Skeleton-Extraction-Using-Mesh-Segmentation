#ifndef MATERIAL_H
#define MATERIAL_H

#include <glm/vec3.hpp>

#include "shader.h"

class Material {
public:
  Material(glm::vec3 ambient, glm::vec3 diffuse, glm::vec3 specular,
           float shininess)
      : ambient{ambient}, diffuse{diffuse}, specular{specular},
        shininess{shininess} {}

  void setUniforms(Shader *shader) const {
    glUniform3f(glGetUniformLocation(shader->programID, "material.ambient"),
                ambient.x, ambient.y, ambient.z);
    glUniform3f(glGetUniformLocation(shader->programID, "material.diffuse"),
                diffuse.x, diffuse.y, diffuse.z);
    glUniform3f(
        glGetUniformLocation(shader->programID, "material.specular"),
        specular.x, specular.y,
        specular
            .z); // Specular doesn't have full effect on this object's material
    glUniform1f(glGetUniformLocation(shader->programID, "material.shininess"),
                shininess);
  }

private:
  glm::vec3 ambient{1};
  glm::vec3 diffuse{1};
  glm::vec3 specular{1};
  float shininess{32};
};

#endif // MATERIAL_H
