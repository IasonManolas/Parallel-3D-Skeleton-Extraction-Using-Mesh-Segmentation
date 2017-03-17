#ifndef LIGHT_H
#define LIGHT_H

#include <glm/vec3.hpp>
#include <QVector2D>
class Light
{
public:
    explicit Light(glm::vec3 lightColor,glm::vec3 position,glm::vec3 direction):ambient{lightColor},diffuse{lightColor},specular{lightColor},position{position},direction{direction}
    {}


protected:
    glm::vec3 ambient{1};
    glm::vec3 diffuse{1};
    glm::vec3 specular{1};
    glm::vec3 position;
    glm::vec3 direction;

};

#endif // LIGHT_H
