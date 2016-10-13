#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //glm::lookAt()

class Camera
{
public:
    Camera(){}
    Camera(const glm::vec3 &pos,const glm::vec3 &target,const glm::vec3 &up)
    {
        this->pos=pos;
        this->target=target;
        this->up=up;
        front=glm::normalize(target-pos);
        right=glm::normalize(glm::cross(front,up));
    }

    glm::mat4 getViewMat()
    {
        return glm::lookAt(pos,target,up);
    }

    glm::vec3 getPos() const;
    void setPos(const glm::vec3 &value);

    glm::vec3 getTarget() const;
    void setTarget(const glm::vec3 &value);

    glm::vec3 getFront() const;
    void setFront(const glm::vec3 &value);

    glm::vec3 getRight() const;
    void setRight(const glm::vec3 &value);

    glm::vec3 getUp() const;
    void setUp(const glm::vec3 &value);

private:
    glm::vec3 pos;
    glm::vec3 target;
    glm::vec3 front;
    glm::vec3 right;
    glm::vec3 up;

    glm::mat4 view;
};


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

inline glm::vec3 Camera::getFront() const
{
return front;
}

inline void Camera::setFront(const glm::vec3 &value)
{
front = value;
}

inline glm::vec3 Camera::getRight() const
{
return right;
}

inline void Camera::setRight(const glm::vec3 &value)
{
right = value;
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
