#ifndef MODEL_H
#define MODEL_H

#include <glm/mat4x4.hpp>

class Model
{
public:

    virtual void Draw()=0;
    virtual void translateTo()=0;

protected:
    glm::mat4 modelMatrix;

};

#endif // MODEL_H
