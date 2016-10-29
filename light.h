#ifndef LIGHT_H
#define LIGHT_H

#include <glm/vec3.hpp>

class Light
{
public:
    explicit Light(glm::vec3 lightColor):ambient{lightColor},diffuse{lightColor},specular{lightColor}
    {}

protected:
    glm::vec3 ambient{1};
    glm::vec3 diffuse{1};
    glm::vec3 specular{1};

};

#endif // LIGHT_H
