#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "directionallight.h"
#include "model.h"
#include "camera.h"

class Scene
{
public:
    Scene();

private:
    DirectionalLight light;
    std::vector<Model> model;
    Camera camera;
};

#endif // SCENE_H
