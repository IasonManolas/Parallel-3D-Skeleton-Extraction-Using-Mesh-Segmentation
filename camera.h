#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //glm::lookAt()
#include <glm/gtx/rotate_vector.hpp> //glm::rotate

class Camera
{
public:
    Camera(){}
    Camera(const glm::vec3 &pos,const glm::vec3 &target,const glm::vec3 &up)
    {
        this->pos=pos;
        this->target=target;
        this->up=up;
    }

    glm::mat4 getViewMatrix()
    {
        return glm::lookAt(pos,target,up);
    }

    void processMouseMovement(const glm::vec2& mouseOffsetVec);
    void processWheelMovement(const int wheelDelta);
    glm::vec3 getPos() const;
    void setPos(const glm::vec3 &value);

    glm::vec3 getTarget() const;
    void setTarget(const glm::vec3 &value);

    glm::vec3 getUp() const;
    void setUp(const glm::vec3 &value);

private:
    glm::vec3 pos;
    glm::vec3 target;
    glm::vec3 up;

};


inline void Camera::processMouseMovement(const glm::vec2& mouseOffsetVec)
{
    glm::vec3 rotationAxis(-mouseOffsetVec.y,-mouseOffsetVec.x,0.0);
    float angle=rotationAxis.length()/100.0;
    rotationAxis=glm::normalize(rotationAxis);
    this->setPos(glm::rotate(this->getPos(),angle,rotationAxis));
}

inline void Camera::processWheelMovement(const int wheelDelta)
{
    if(wheelDelta>0)
    this->setPos(this->getPos()+0.1f*glm::normalize(this->getTarget()-this->getPos()));
    else
    this->setPos(this->getPos()-0.1f*glm::normalize(this->getTarget()-this->getPos()));

}

inline glm::vec3 Camera::getPos() const
{
return pos;
}

inline void Camera::setPos(const glm::vec3 &value)
{
pos = value;
}

inline glm::vec3 Camera::getTarget() const
{
return target;
}

inline void Camera::setTarget(const glm::vec3 &value)
{
target = value;
}

inline glm::vec3 Camera::getUp() const
{
return up;
}

inline void Camera::setUp(const glm::vec3 &value)
{
up = value;
}

#endif // CAMERA_H
