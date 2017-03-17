#ifndef RAY_CAST_PICKING_H
#define RAY_CAST_PICKING_H
#include <glm/vec3.hpp>

inline glm::vec3 getNormalisedDeviceCoordinates(int mouseX,int mouseY,int width,int height)
{
    float x=2.0*mouseX/width-1;
    float y=1-(2.0*mouseY)/height;
    float z=-1; //we want the ray to point into the screen
    return glm::vec3(x,y,z);
}

#endif // RAY_CAST_PICKING_H
