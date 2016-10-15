#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //glm::lookAt()
#include <glm/gtx/rotate_vector.hpp> //glm::rotat
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <QVector2D>
#include <QVector3D>
class Camera
{
public:
    Camera(){}
    Camera(const glm::vec3 &pos,const glm::vec3 &target,const glm::vec3 &up)
    {
        this->position=pos;
        this->target=target;
        this->up=glm::normalize(up);
        viewMat=glm::lookAt(position,target,up);

        rotation=glm::quat();
        glm::normalize(rotation);
    }

    glm::mat4 getViewMatrix()
    {
        return viewMat;
    }

    void processMouseMovement(const QVector2D& mouseOffsetVec);
    void processWheelMovement(const int wheelDelta);
    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3 &value);

    glm::vec3 getTarget() const;
    void setTarget(const glm::vec3 &value);

    glm::vec3 getUp() const;
    void setUp(const glm::vec3 &value);

private:
    glm::vec3 position;
    glm::vec3 target;
    glm::vec3 up;
    glm::vec3 look;
    glm::vec3 right;
    glm::mat4 viewMat;

    glm::quat rotation; //this quaternion represents the orientation of the cam?

};


inline void Camera::processMouseMovement(const QVector2D& mouseDV)
{
//  glm::mat4 R=glm::yawPitchRoll(mouseDV.x/100.0,mouseDV.y/.100,0.0);
//  glm::vec3 T=glm::vec3(position.x,position.y,position.z);
//  T=glm::vec3(R*glm::vec4(T,0.0f));
//  position=target+T;
//    std::cout<<"position.x="<<position.x<<" position.y="<<position.y<<" position.z="<<position.z<<std::endl;

//  look=glm::normalize(target-position);
//  up=glm::vec3(R*glm::vec4(0.0f,1.0f,0.0f,0.0f));
//  right=glm::cross(look,up);
  viewMat=glm::lookAt(position,target,up);
    QVector3D rotationAxis(mouseDV.y(),mouseDV.x(),0.0);
    float angle=mouseDV.length()/100.0;

    glm::quat rot=glm::angleAxis(angle,glm::vec3(rotationAxis.x(),rotationAxis.y(),0.0f));
    rotation=rot*rotation;
    viewMat=viewMat*glm::toMat4(rotation);
}

inline void Camera::processWheelMovement(const int wheelDelta)
{
    if(wheelDelta>0)
    this->setPosition(this->getPosition()+0.1f*glm::normalize(this->getTarget()-this->getPosition()));
    else
    this->setPosition(this->getPosition()-0.1f*glm::normalize(this->getTarget()-this->getPosition()));

}

inline glm::vec3 Camera::getPosition() const
{
return position;
}

inline void Camera::setPosition(const glm::vec3 &value)
{
position = value;
//up=glm::normalize(target-position)
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
