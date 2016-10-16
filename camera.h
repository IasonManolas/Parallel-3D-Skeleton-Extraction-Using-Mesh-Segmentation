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
        this->worldUp=glm::normalize(up);
        this->up=worldUp;

        right=glm::normalize(glm::cross(look,worldUp));
        viewMat=glm::lookAt(position,target,up);

        rotation=glm::quat();
    }

    void processMouseMovement(const QVector2D& mouseDV)
    {
        glm::vec3 rotationAxis(mouseDV.y(),mouseDV.x(),0.0);
        float angle=rotationAxis.length()/200.0;
        rotationAxis=glm::normalize(rotationAxis);

        glm::quat rot=glm::angleAxis(angle,rotationAxis);
        rotation=rot*rotation;
        glm::mat4 rotationMatrix=glm::toMat4(rotation);

        viewMat=glm::lookAt(position,target,up);
        viewMat=viewMat*rotationMatrix;

     }


    void processWheelMovement(const int& wheelDelta)
    {
        if(wheelDelta>0)
        position+=0.1f*glm::normalize(target-position);
        else
        position-=0.1f*glm::normalize(target-position);
        viewMat=glm::lookAt(position,target,up);
    }



    glm::vec3 getPosition() const
    {
        return position;
    }
    void setPosition(const glm::vec3 &value)
    {
        position=value;
    }

    glm::mat4 getViewMat() const
    {
       return viewMat;
    }

private:
    glm::vec3 position;
    glm::vec3 target;

    glm::vec3 look;
    glm::vec3 worldUp;
    glm::vec3 up;
    glm::vec3 right;

    glm::mat4 viewMat;

    glm::quat rotation; //this quaternion represents the orientation of the cam?

};
#endif // CAMERA_H
