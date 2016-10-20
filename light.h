#ifndef LIGHT_H
#define LIGHT_H

#include <glm/vec3.hpp>

class Light
{
public:
    Light():ambient{1.0f},diffuse{1.0f},specular{1.0f}
    {}

protected:
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;

};

#endif // LIGHT_H
